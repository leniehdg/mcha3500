#include "IMU.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include <stdint.h> 
#include "tm_stm32_mpu6050.h"
#include <math.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

/* Variable declarations */
TM_MPU6050_t IMU_datastruct;

/* Function defintions */
void IMU_init(void)
{
    /* TODO: Initialise IMU with AD0 LOW, acceleration sensitivity +-4g, gyroscope +-250 deg/s */
    /* Initialise I2C communication */
    
    TM_I2C_Init(I2C1, TM_I2C_PinsPack_1, 400000);
    TM_MPU6050_Init	(&IMU_datastruct, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_4G, TM_MPU6050_Gyroscope_250s);	
}

 void IMU_read(void)
{
    /* TODO: Read all IMU values */
    TM_MPU6050_ReadAll(&IMU_datastruct);
}

float get_accX(void)
{
    IMU_read();

    /* TODO: Convert acceleration reading to ms^-2 */
    float X_acc = IMU_datastruct.Accelerometer_X/8192.0;
	X_acc = X_acc * 9.81;
    
    /* TODO: return the Y acceleration */
    return X_acc;
}

float get_accZ(void)
{
    IMU_read();

    /* Convert acceleration reading to ms^-2 */
    int16_t rawAccZ = IMU_datastruct.Accelerometer_Z;
    float sensitivity = 8192.0; // ±4g sensitivity for ±32768 range
    
    float accZ_ms2 = (rawAccZ / sensitivity) * 9.81; // 9.81 m/s^2 is the acceleration due to gravity
    
    /* Return the Z acceleration */
    return accZ_ms2;
}

float get_gyroY(void)
{
    IMU_read();

    /* Convert gyro reading to radians per second */
    int16_t rawGyroY = IMU_datastruct.Gyroscope_Y;
    float sensitivity = 131.0; // Sensitivity for ±250°/s range
    float GyroY_rad = (rawGyroY / sensitivity) * (M_PI / 180.0); // Convert to radians/s
    
    /* Return the X angular velocity */
    return GyroY_rad;
}


// float get_gyroX(void)
// {
//     IMU_read();
    
//     /* Convert gyro reading to radians per second */
//     int16_t rawGyroX = IMU_datastruct.Gyroscope_X;
//     float sensitivity = 131.0; // Sensitivity for ±250°/s range
//     float GyroX_rad = (rawGyroX / sensitivity) * (M_PI / 180.0); // Convert to radians/s
    
//     /* Return the X angular velocity */
//     return GyroX_rad;
// }

float get_acc_angle(void)
{
    float accZ_ms2 = get_accZ();
    float accX_ms2 = get_accX();
    /* Compute IMU angle using accY and accZ using atan2 */
    float imu_angle = -atan2(accX_ms2, accZ_ms2) + 1.59;   // whoopsie offset (pi/2)

    /*  return the IMU angle*/
    return imu_angle;
}