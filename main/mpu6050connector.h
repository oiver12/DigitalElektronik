#ifndef AAA_C_CONNECTOR_H 
#define AAA_C_CONNECTOR_H 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef struct _mpu6050_acceleration_t
{
    float accel_x;
    float accel_y;
    float accel_z;
} mpu6050_acceleration_t;

typedef struct _mpu6050_rotation_t
{
    float yaw;
    float pitch;
    float roll;
} mpu6050_rotation_t;

void mpuISR(void*);
void mpuTask(void* pvParameters);

void MPU6050_SetBus(uint32_t CLOCK_SPEED);
void MPU6050_SetAdress();
esp_err_t MPU6050_testConnection();
esp_err_t MPU6050_Initialize();
esp_err_t MPU6050_GetAcceleration(mpu6050_acceleration_t* axes);

#ifdef __cplusplus
}
#endif


#endif