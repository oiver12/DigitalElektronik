#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include <esp_http_server.h>
#include "mpu6050connector.h"
#include <driver/i2c.h>
#include "driver/ledc.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "driver/mcpwm.h"
#include "PID.h"

/* Controller parameters */
#define PID_KP  3.0f
#define PID_KI  4
#define PID_KD  9

#define PID_TAU 0.02f

#define PID_LIM_MIN -20.0f
#define PID_LIM_MAX  20.0f

#define PID_LIM_MIN_INT -10.0f
#define PID_LIM_MAX_INT  10.0f


#define PORT                        3333
#define KEEPALIVE_IDLE              5
#define KEEPALIVE_INTERVAL          5
#define KEEPALIVE_COUNT             3

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (26) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
//#define LEDC_DUTY               (1024) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (1000) // Frequency in Hertz. Set frequency at 5 kHz
#define BIN1                     15
#define BIN2                     14
#define STBY                     34

#define SERVO_MIN_PULSEWIDTH_US (1000) // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US (2000) // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE        (90)   // Maximum angle in degree upto which servo can rotate

#define SERVO_PULSE_GPIO        (18)   // GPIO connects to the PWM signal line

PIDController pidController = {.Kp = PID_KP, .Ki = PID_KI, .Kd = PID_KD,
                        .tau = PID_TAU,
                        .limMin = PID_LIM_MIN, .limMax = PID_LIM_MAX,
			            .limMinInt = PID_LIM_MIN_INT, .limMaxInt = PID_LIM_MAX_INT,
                        .T = 0.1 };
int LEDC_DUTY = 0;
int startDuty = 0;
int desiredDuty = 0;
int angle = 0;
TaskHandle_t dutyMotorHandle = NULL;
static const char *TAG = "main";
void printTask(void*);
bool motorRestart = false;

SemaphoreHandle_t motorRestart_mutex = NULL;

bool MotorToDesired(int desired)
{
    int start = LEDC_DUTY;
    int stepSize = 50; //50*20 = 1000 pro Sekunde --> 12% von ganzem
    ESP_LOGI(TAG, "Started Motor Routine. Steps: %i, Desired: %i, Start: %i", (int)((abs(desired-start))/stepSize), desired, start);
    for (size_t i = 1; i < (int)((abs(desired-start))/stepSize) + 1; i++)
    {
        bool restart = false;
        xSemaphoreTake(motorRestart_mutex, portMAX_DELAY);
        {
            restart = motorRestart;
        }
        xSemaphoreGive(motorRestart_mutex);
        if(restart)
        {
            ESP_LOGI(TAG, "Restarted");
            return false;
        }
        if(start < desired)
        {
            LEDC_DUTY = start + stepSize * i;
        }
        else
        {
            LEDC_DUTY = start - stepSize * i;
        }
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        vTaskDelay(10 / portTICK_PERIOD_MS); //all 100ms -> 10 Hz
    }
    return true;
}

void setDutyCycleMotor()
{
    bool isBackwards = false;
    while(true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xSemaphoreTake(motorRestart_mutex, portMAX_DELAY);
        {
            motorRestart = false;
        }
        xSemaphoreGive(motorRestart_mutex);
        if(isBackwards)
        {
            startDuty = -startDuty;
        }
        if(desiredDuty < 0 && startDuty > 0)
        {
            ESP_LOGI(TAG, "1: Start: %i, End: %i.", startDuty, desiredDuty);
            //vorwärts langsamer werden
            if(!MotorToDesired(0))
            {
                continue;
            }
            isBackwards = true;
            //gpio_set_level(BIN1, 0);
            //gpio_set_level(BIN2, 1);
            MotorToDesired(-desiredDuty);
        }
        else if(desiredDuty > 0 && startDuty < 0)
        {
            ESP_LOGI(TAG, "2: Start: %i, End: %i.", startDuty, desiredDuty);
            if(!MotorToDesired(0))
            {
                continue;
            }
            isBackwards = false;
            //gpio_set_level(BIN1, 1);
            //gpio_set_level(BIN2, 0);
            MotorToDesired(desiredDuty);
        }
        else
        {
            ESP_LOGI(TAG, "3: Start: %i, End: %i.", startDuty, desiredDuty);
            MotorToDesired(abs(desiredDuty));
        }
        
    }
    vTaskDelete(NULL);
}

//für Motor
static void start_pwm(void)
{
     // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void tcp_message(const int sock)
{
    int len;
    char rx_buffer[128];

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);
            //empfangene string mit 's' = speed und Angle von Joystick
            if(rx_buffer[0] == 's')
            {
                int i = 0;
                sscanf(rx_buffer ,"s%d %d", &i, &angle);
                ESP_LOGI(TAG, "Speed %d, Angle %d", i, angle);
                desiredDuty = i;
                startDuty = LEDC_DUTY;
                xSemaphoreTake(motorRestart_mutex, portMAX_DELAY);
                {
                    motorRestart = true;
                }
                xSemaphoreGive(motorRestart_mutex);
                xTaskNotifyGive(dutyMotorHandle);
            }
            //nur Angel
            else if(rx_buffer[0] == 'a')
            {
                sscanf(rx_buffer, "a %d", &angle);
                ESP_LOGI(TAG,"Angle %d \n", angle);
            }
            //PID Controller für Konifguration
            else if(rx_buffer[0] == 'p')
            {
                int pCo = 0;
                int iCo = 0;
                int dCo = 0;
                int limCo = 0;
                sscanf(rx_buffer, "p %d, %d, %d, %d", &pCo, &iCo, &dCo, &limCo);
                pidController.Kp = (float)pCo / 100.0f;
                pidController.Ki = (float)iCo / 100.0f;
                pidController.Kd = (float)dCo / 100.0f;
                pidController.limMinInt = (float)limCo / -100.0f;
                pidController.limMaxInt = (float)limCo / 100.0f;
                PIDController_Init(&pidController);
                ESP_LOGI(TAG,"P %.3f, I %.3f, D %.3f, L %.3f", pidController.Kp, pidController.Ki, pidController.Kd, pidController.limMaxInt);
            }
        }
    } while (len > 0);
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }
#ifdef CONFIG_EXAMPLE_IPV6
    else if (addr_family == AF_INET6) {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(PORT);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
#ifdef CONFIG_EXAMPLE_IPV6
        else if (source_addr.ss_family == PF_INET6) {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        tcp_message(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

static inline uint32_t example_convert_servo_angle_to_duty_us(int angle)
{
    return (angle + SERVO_MAX_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (2 * SERVO_MAX_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

//Servo zu desired Angle und PID Controller
void ServoTask(void *pvParameters)
{
    // float multiplier = 0.5; //45 --> rotationSum 5
    // float nullPoint = -2; //wo der Servo stehenbliebt: nicht 0 sondern -2
    // float maxRotations = 0.8; //max Auslenkung vom Gewicht
    // float rotationsPerSecMaxSpeed = 0.666;
    // float rotationsSum = 0;
    // float lastTime = esp_timer_get_time();
    // float servoSpeed = 0;
    //  /* Initialise PID controller */
    // PIDController_Init(&pidController);
    // //TODO: Race Conditions?
    // mpu6050_rotation_t *rotations = (mpu6050_rotation_t *)pvParameters;
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    // ESP_LOGI(TAG, "Start Servo");
    // while(true)
    // {
    //     float deltaT = (esp_timer_get_time()-lastTime) / 1000000;
    //     //clamp servoSpeed zwischen -1 und 1. Multipliziere mit rotationsPerSecMaxSpeed deltaT, um gemachte Rotations seit letzter Rechnung ausrechnen.
    //     rotationsSum += (servoSpeed / PID_LIM_MAX) * rotationsPerSecMaxSpeed * (deltaT);
    //     float desiredRotationsSum = angle / (90 * multiplier) * maxRotations;
    //     //float desiredRotationsSum = rotations->roll / (90 * multiplier) * maxRotations;
    //     //ESP_LOGI(TAG, "Des %.3f, Rotation %.3f", desiredRotationsSum, rotationsSum);
    //     pidController.T = deltaT;
    //     PIDController_Update(&pidController, desiredRotationsSum, rotationsSum);
    //     servoSpeed = pidController.out;
    //     //servoSpeed = (desiredRotationsSum - rotationsSum) / maxRotations * servoAngleMaxSpeed; //ab 5 Differenz dreht der Motor ganz schnell
    //     lastTime = esp_timer_get_time();
    //     if(servoSpeed > PID_LIM_MAX)
    //     {
    //         servoSpeed = PID_LIM_MAX;
    //     }
    //     else if(servoSpeed < PID_LIM_MIN)
    //     {
    //         servoSpeed = PID_LIM_MIN;
    //     }
    //     if(rotationsSum > maxRotations || rotationsSum < -maxRotations)
    //     {
    //         if(rotationsSum > maxRotations)
    //         {
    //             rotationsSum = maxRotations - 0.05;
    //         }
    //         else if(rotationsSum < -maxRotations)
    //         {
    //             rotationsSum = -maxRotations + 0.05;
    //         }
    //         //wenn voll gedreht, nicht mehr drehen.
    //         servoSpeed = 0;
    //     }
    //     ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, example_convert_servo_angle_to_duty_us(servoSpeed+nullPoint)));
    //     vTaskDelay(100 / portTICK_PERIOD_MS);
    // }
    //von Sensor
    mpu6050_rotation_t *rotations = (mpu6050_rotation_t *)pvParameters;
    PIDController_Init(&pidController);
    float lastTime = esp_timer_get_time();
    const float kugelAngleGerade = 0;
    while(true)
    {
        float deltaT = (esp_timer_get_time()-lastTime) / 1000000;
        // pidController.T = deltaT;
        // PIDController_Update(&pidController, kugelAngleGerade, angle);
        // float servoAngle = pidController.out;
        // ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, example_convert_servo_angle_to_duty_us(servoAngle)));
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, example_convert_servo_angle_to_duty_us(angle)));
        lastTime = esp_timer_get_time();
        vTaskDelay(400 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    //Mutex für Multithread
    motorRestart_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 7, NULL);

    mpu6050_rotation_t rot = {
        .pitch = 0,
        .roll = 0,
        .yaw = 0
    };
    //Sensor auslesen Task
    xTaskCreate(mpuTask, "mpuTask", 4 * 1024, &rot, 6, NULL);
    xTaskCreate(printTask, "printTask", 2 * 1024, &rot, 5, NULL);
    //Motor Init GPIO
    gpio_config_t io_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL<<BIN1) | (1ULL<<BIN2) | (1ULL<<STBY)),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_config);
    gpio_set_level(BIN1, 1);
    gpio_set_level(BIN2, 0);
    gpio_set_level(STBY, 1);
    //Start Motor PWM
    start_pwm();
    printf("Started MotorPins \n");
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    printf("Started PWM \n");
    xTaskCreate(setDutyCycleMotor, "MotorTask", 2 * 1024, NULL, 2, &dutyMotorHandle);

    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, SERVO_PULSE_GPIO); // To drive a RC servo, one MCPWM generator is enough
    //PWM für Servo
    mcpwm_config_t pwm_config = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

    xTaskCreate(ServoTask, "servoTask", 2 * 1024, &rot, 5, NULL);
}

//Für Debug
void printTask(void* ps)
{
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    mpu6050_rotation_t *rotations = (mpu6050_rotation_t *)ps;
    while (true) {
        printf("Pitch: %+6.1f \t Roll: %+6.1f \t Yaw: %+6.1f \n", rotations->pitch, rotations->roll, rotations->yaw);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
