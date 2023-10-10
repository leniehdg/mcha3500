#include <stdint.h> 
#include "stm32f4xx_hal.h" // to import UNUSED() macro
#include "cmsis_os2.h" // Include CMSIS-RTOS header
#include "uart.h"

// Modules that provide commands
#include "pendulum.h"
#include "data_logging.h"
#include "IMU.h"
#include "encoder.h"
#include "tm_stm32_mpu6050.h"

/* Variable declarations */
uint16_t logCount;
int32_t encoder_count = 0;
osTimerId_t timerHandle; // Timer handle
static void (*log_function) (void);

/* Function declarations */
// Basic Log Commands
static void log_pointer(void *argument);

// Pendulum
static void log_pendulum(void *argument);

// Encoder
static void log_encoder(void);

// IMU 
static void log_imu(void);


/* ------------------------------------  BASIC LOG COMMANDS  -------------------------- */

void logging_init(void)
{
    /* TODO: Initialize any necessary peripherals and resources */
    /* Create a timer with 5ms interval (200 Hz) */
    const osTimerAttr_t timerAttr = {
        .name = "DataLoggingTimer"
    };
    timerHandle = osTimerNew(log_pointer, osTimerPeriodic, NULL, &timerAttr);
    
}

static void log_pointer(void *argument)
{
    UNUSED(argument);

    /* call function pointed to by log_function */
    (*log_function)();
}

void logging_start(void)
{
    /* Reset the logCount variable */
    logCount = 0;

    /* function pointer to pend logging func */
    log_function = &log_encoder;    //   &log_imu;    // &log_pendulum;

    /* Start the data logging timer */
    osStatus_t status = osTimerStart(timerHandle, 5); // 5 ms interval (timerPeriod)
    
}

void logging_stop(void)
{
    /* Stop the data logging timer */
    osStatus_t status = osTimerStop(timerHandle);
    
}


/* ------------------------------------  PENDULUM  -------------------------- */

static void log_pendulum(void *argument)
{
    

    /* TODO: Read the potentiometer voltage */
    float voltage = pendulum_read_voltage();        
    float time = logCount/200.0;

    /* TODO: Print the sample time and potentiometer voltage to the serial terminal in the format [time],[
    voltage] */
    printf("%.4f, %f\n", time, voltage);        

    /* TODO: Increment log count */
    logCount++;

    /* TODO: Stop logging once 2 seconds is reached  */
    if (logCount >= 400) { // Assuming 5 ms interval, 400 intervals = 2000 ms (2 seconds)
        logging_stop();
    }
    
    //return time;
}   


/* ------------------------------------  IMU  -------------------------- */


static void log_imu(void)
{
    /* Get the imu angle from accelerometer readings */
    float accelerometer_angle = get_acc_angle();
    
    /* Get the imu X gyro reading */
    float gyro_angular_velocity = get_gyroX();
    
    /* Read the potentiometer voltage */
    float voltage = pendulum_read_voltage();
    
    float time = logCount / 200.0;

    /* Print the time, accelerometer angle, gyro angular velocity, and pot voltage values */
    printf("%.3f,%.3f,%.3f,%f\n", time, accelerometer_angle, gyro_angular_velocity, voltage);

    /* Increment log count */
    logCount++;

    /* Stop logging once 5 seconds is reached */
    if (logCount >= 1000) // 5 seconds at 200 Hz
    {
        logging_stop();
    }
}


/* ------------------------------------  ENCODER  -------------------------- */

static void log_encoder(void)
{
    // Read encoder data
    int32_t encoder_count = motor_encoder_getValue();

    // Print encoder data
    printf("%f, %ld\n", logCount, encoder_count);

    /* TODO: Increment log count */
    logCount++;
    
    /* TODO: Stop logging once 5 seconds is reached */
    if(logCount >=1000)
    {
        logging_stop();
    }
}