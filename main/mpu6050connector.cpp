//https://github.com/natanaeljr/esp32-MPU-driver/tree/master/examples

#include "mpu6050connector.h"
#include "I2Cbus.hpp"
#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"
#include <esp_log.h>

#ifdef __cplusplus
extern "C" {
#endif
static const char *TAG = "MPU6050Conn";
// Inside this "extern C" block, I can implement functions in C++, which will externally 
//   appear as C functions (which means that the function IDs will be their names, unlike
//   the regular C++ behavior, which allows defining multiple functions with the same name
//   (overloading) and hence uses function signature hashing to enforce unique IDs),

static constexpr int kInterruptPin         = 17;  // GPIO_NUM
static constexpr uint16_t kSampleRate      = 250;  // Hz
static constexpr mpud::accel_fs_t kAccelFS = mpud::ACCEL_FS_4G;
static constexpr mpud::gyro_fs_t kGyroFS   = mpud::GYRO_FS_500DPS;
static constexpr mpud::dlpf_t kDLPF        = mpud::DLPF_98HZ;
static constexpr mpud::int_config_t kInterruptConfig{
    .level = mpud::INT_LVL_ACTIVE_HIGH,
    .drive = mpud::INT_DRV_PUSHPULL,
    .mode  = mpud::INT_MODE_PULSE50US,
    .clear = mpud::INT_CLEAR_STATUS_REG  //
};

static MPU_t *MPUInstance = NULL;
static mpud::raw_axes_t accelRaw;
static mpud::float_axes_t accelG;

void lazyMPU() {
    if (MPUInstance == NULL) {
        MPUInstance = new mpud::MPU();
    }
}

void MPU6050_SetBus(uint32_t CLOCK_SPEED)
{
    i2c0.begin(GPIO_NUM_21, GPIO_NUM_22, CLOCK_SPEED);
    lazyMPU();
    MPUInstance->setBus(i2c0);
}
void MPU6050_SetAdress()
{
    MPUInstance->setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);
}
esp_err_t MPU6050_testConnection()
{
    return MPUInstance->testConnection();
}
esp_err_t MPU6050_Initialize()
{
    return MPUInstance->initialize();
}
esp_err_t MPU6050_GetAcceleration(mpu6050_acceleration_t* axes)
{
    esp_err_t err = MPUInstance->acceleration(&accelRaw);
    accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
    axes->accel_x = accelG.x;
    axes->accel_y = accelG.y;
    axes->accel_z = accelG.z;
    return err;
}

void mpuTask(void* pvParameters)
{
    mpu6050_rotation_t *rotations = (mpu6050_rotation_t *)pvParameters;
    i2c0.begin(GPIO_NUM_21, GPIO_NUM_22, 400000);
    lazyMPU();
    MPUInstance->setBus(i2c0);
    MPUInstance->setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);
    // Verify connection
    while (esp_err_t err = MPUInstance->testConnection()) {
        ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU connection successful!");

    // Initialize
    ESP_ERROR_CHECK(MPUInstance->initialize());

    // Self-Test
    mpud::selftest_t retSelfTest;
    while (esp_err_t err = MPUInstance->selfTest(&retSelfTest)) {
        ESP_LOGE(TAG, "Failed to perform MPU Self-Test, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU Self-Test result: Gyro=%s Accel=%s",  //
             (retSelfTest & mpud::SELF_TEST_GYRO_FAIL ? "FAIL" : "OK"),
             (retSelfTest & mpud::SELF_TEST_ACCEL_FAIL ? "FAIL" : "OK"));

    // Calibrate
    mpud::raw_axes_t accelBias, gyroBias;
    ESP_ERROR_CHECK(MPUInstance->computeOffsets(&accelBias, &gyroBias));
    ESP_ERROR_CHECK(MPUInstance->setAccelOffset(accelBias));
    ESP_ERROR_CHECK(MPUInstance->setGyroOffset(gyroBias));
     // Configure
    ESP_ERROR_CHECK(MPUInstance->setAccelFullScale(kAccelFS));
    ESP_ERROR_CHECK(MPUInstance->setGyroFullScale(kGyroFS));
    ESP_ERROR_CHECK(MPUInstance->setSampleRate(kSampleRate));
    ESP_ERROR_CHECK(MPUInstance->setDigitalLowPassFilter(kDLPF));

    // Setup FIFO
    ESP_ERROR_CHECK(MPUInstance->setFIFOConfig(mpud::FIFO_CFG_ACCEL | mpud::FIFO_CFG_GYRO));
    ESP_ERROR_CHECK(MPUInstance->setFIFOEnabled(true));
    constexpr uint16_t kFIFOPacketSize = 12;

    // Setup Interrupt
    constexpr gpio_config_t kGPIOConfig{
        .pin_bit_mask = (uint64_t) 0x1 << kInterruptPin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE  //
    };
    gpio_config(&kGPIOConfig);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add((gpio_num_t) kInterruptPin, mpuISR, xTaskGetCurrentTaskHandle());
    ESP_ERROR_CHECK(MPUInstance->setInterruptConfig(kInterruptConfig));
    ESP_ERROR_CHECK(MPUInstance->setInterruptEnabled(mpud::INT_EN_RAWDATA_READY));

    // Ready to start reading
    ESP_ERROR_CHECK(MPUInstance->resetFIFO());  // start clean

    // Reading Loop
    while (true) {
        // Wait for notification from mpuISR
        uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (notificationValue > 1) {
            ESP_LOGW(TAG, "Task Notification higher than 1, value: %d", notificationValue);
            MPUInstance->resetFIFO();
            continue;
        }
        // Check FIFO count
        uint16_t fifocount = MPUInstance->getFIFOCount();
        if (esp_err_t err = MPUInstance->lastError()) {
            ESP_LOGE(TAG, "Error reading fifo count, %#X", err);
            MPUInstance->resetFIFO();
            continue;
        }
        if (fifocount > kFIFOPacketSize * 2) {
            if (!(fifocount % kFIFOPacketSize)) {
                ESP_LOGE(TAG, "Sample Rate too high!, not keeping up the pace!, count: %d", fifocount);
            }
            else {
                ESP_LOGE(TAG, "FIFO Count misaligned! Expected: %d, Actual: %d", kFIFOPacketSize, fifocount);
            }
            MPUInstance->resetFIFO();
            continue;
        }
        // Burst read data from FIFO
        uint8_t buffer[kFIFOPacketSize];
        if (esp_err_t err = MPUInstance->readFIFO(kFIFOPacketSize, buffer)) {
            ESP_LOGE(TAG, "Error reading sensor data, %#X", err);
            MPUInstance->resetFIFO();
            continue;
        }
        // Format
        mpud::raw_axes_t rawAccel, rawGyro;
        rawAccel.x = buffer[0] << 8 | buffer[1];
        rawAccel.y = buffer[2] << 8 | buffer[3];
        rawAccel.z = buffer[4] << 8 | buffer[5];
        rawGyro.x  = buffer[6] << 8 | buffer[7];
        rawGyro.y  = buffer[8] << 8 | buffer[9];
        rawGyro.z  = buffer[10] << 8 | buffer[11];
        // Calculate tilt angle
        // range: (roll[-180,180]  pitch[-90,90]  yaw[-180,180])
        constexpr double kRadToDeg = 57.2957795131;
        constexpr float kDeltaTime = 1.f / kSampleRate;
        float gyroRoll             = rotations->roll + mpud::math::gyroDegPerSec(rawGyro.x, kGyroFS) * kDeltaTime;
        float gyroPitch            = rotations->pitch + mpud::math::gyroDegPerSec(rawGyro.y, kGyroFS) * kDeltaTime;
        float gyroYaw              = rotations->yaw + mpud::math::gyroDegPerSec(rawGyro.z, kGyroFS) * kDeltaTime;
        float accelRoll            = atan2(-rawAccel.x, rawAccel.z) * kRadToDeg;
        float accelPitch = atan2(rawAccel.y, sqrt(rawAccel.x * rawAccel.x + rawAccel.z * rawAccel.z)) * kRadToDeg;
        // Fusion
        rotations->roll  = gyroRoll * 0.95f + accelRoll * 0.05f;
        rotations->pitch = gyroPitch * 0.95f + accelPitch * 0.05f;
        rotations->yaw   = gyroYaw;
        // correct yaw
        if (rotations->yaw > 180.f)
            rotations->yaw -= 360.f;
        else if (rotations->yaw < -180.f)
            rotations->yaw += 360.f;
    }
    vTaskDelete(nullptr);
}

IRAM_ATTR void mpuISR(TaskHandle_t taskHandle)
{
    BaseType_t HPTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandle, &HPTaskWoken);
    if (HPTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

#ifdef __cplusplus
}
#endif