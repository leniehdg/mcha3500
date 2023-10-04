#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "IMU.h"
#include "tm_stm32_mpu6050.h"
#include "math.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Variable declarations */
TM_MPU6050_t IMU_datastruct;
float accY;
float accZ;
float accX;
float accAngle;

/* Function defintions */
void IMU_init(void)
{
    /* TODO: Initialise IMU with AD0 LOW, accelleration sensitivity +-4g, gyroscope +-250 deg/s */
    TM_MPU6050_Init(&IMU_datastruct,TM_MPU6050_Device_0,TM_MPU6050_Accelerometer_4G,TM_MPU6050_Gyroscope_250s);
}

void IMU_read(void)
{
    /* TODO: Read all IMU values */
    TM_MPU6050_ReadAll(&IMU_datastruct);
    //TM_MPU6050_ReadAccelerometer(&IMU_datastruct);
}

float get_accX(void)
{
    /* TODO: Convert accelleration reading to ms^-2. */
    /* TODO: return the Y acceleration */
    accX = IMU_datastruct.Accelerometer_X*4*9.81/32786;
    return accX;
}

float get_accY(void)
{
    /* TODO: Convert accelleration reading to ms^-2. */
    /* TODO: return the Y acceleration */
    accY = IMU_datastruct.Accelerometer_Y*4*9.81/32786;
    return accY;
}

float get_accZ(void)
{
    /* TODO: return the Z acceleration */
    accZ = IMU_datastruct.Accelerometer_Z*4*9.81/32786;
    return accZ;
}

float get_gyroX(void)
{
    /* TODO: return the X angular velocity */
    accX = IMU_datastruct.Gyroscope_X*(250*3.14159265358979323846)/(180*32786.0);
    return accX;
}

///*******************//////////////////// Onboard baby ///*******************////////////////////


float get_gyroZ(void)
{
    /* TODO: return the X angular velocity */
    accZ = -IMU_datastruct.Gyroscope_Z*(250.0*3.14159265358979323846)/(180.0*32786.0);
    //accZ = (IMU_datastruct.Gyroscope_Z/250.0)*((2.0*3.14159265358979323846)/180);
    return accZ;
}

float get_acc_angle(void)
{
    /* TODO: compute IMU angle using accY and accZ using atan2 */
    //accAngle = -atan2(get_accZ(),get_accY());
    accAngle = -atan2(get_accX(),get_accY());
    /* TODO: return the IMU angle */

    return accAngle;
}