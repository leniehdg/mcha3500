#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "data_logging.h"
#include "IMU.h"
#include "encoder.h"
#include "observer.h"
#include "stepper.h"

/* Variable declarations */
uint16_t logCount;
static osTimerId_t logTimer;

/* Variable declarations */
static void (*log_function)(void);

/* Function declarations */
static void log_pointer(void *argument);

static void log_imu(void);

static void log_encoder(void);

static void log_Kfilter(void);


void logging_init(void)
{
   /* TODO: Modify os timer for use with generic logging function */
   logTimer = osTimerNew(log_pointer, osTimerPeriodic, NULL, NULL);
}


static void log_pointer(void *argument)
{
   UNUSED(argument); 
   /* Call function pointed to by log_function */
   (*log_function)();
}

void logging_stop(void)
{
   /* TODO: Stop data logging timer */
    osTimerStop(logTimer);
}

void encoder_logging_start(void)
{
   log_function = &log_encoder;
   /* TODO: Reset the log counter */
   logCount = 0;
   /* TODO: Start data logging timer at 200Hz */
   osTimerStart(logTimer,5);
}

static void log_encoder(void)
{
      /* TODO: Read the potentiometer voltage */
    int32_t enc =  motor_encoder_getValue();
    
      /* TODO: Print the sample time and potentiometer voltage to the serial terminal in the format [time],[voltage] */
    printf("%f,%ld\n", logCount*0.005, enc);
      /* TODO: Increment log count */
    logCount++;
      
      /* TODO: Stop logging once 2 seconds is reached (Complete this once you have created the stop function in the next step) */
    if(logCount == 1000){
        logging_stop();
    }
        
}

void imu_logging_start(void)
{
   /* TODO: Change function pointer to the imu logging function (log_imu) */
   log_function = &log_imu;
   /* TODO: Reset the log counter */
   logCount = 0;
   /* TODO: Start data logging at 200Hz */
   osTimerStart(logTimer,5);
}

static void log_imu(void)
{
   /* TODO: Read IMU */
   IMU_read();
   /* TODO: Get the imu angle from accelerometer readings */
   get_acc_angle();
   /* TODO: Get the imu X gyro reading */
   get_gyroZ();
   /* TODO: Read the potentiometer voltage */
   //pendulum_read_voltage();
   /* TODO: Print the time, accelerometer angle, gyro angular velocity and pot voltage values to the
   serial terminal in the format %f,%f,%f,%f\n */

   printf("%f,%f,%f\n",logCount*0.005,get_acc_angle(),get_gyroZ()); 

   /* TODO: Increment log count */
   logCount++;

   //* TODO: Stop logging once 5 seconds is reached */
   if(logCount == 1000){
        logging_stop();
   }
}

void kfilter_logging_start(void)
{
   /* Testing SS Kflilter */
  log_function = &log_Kfilter;
  //  logCount = 0;

   /* TODO: Start data logging at 200Hz */
   osTimerStart(logTimer,5);
}

static void log_Kfilter(void)
{

  
   /* TODO: Read IMU */
   IMU_read();
   /* TODO: Get the imu angle from accelerometer readings */
   //get_acc_angle();
   /* TODO: Get the imu X gyro reading */
   //get_gyroZ();
   /* TODO: Read the potentiometer voltage */
   //pendulum_read_voltage();
   /* TODO: Print the time, accelerometer angle, gyro angular velocity and pot voltage values to the
   serial terminal in the format %f,%f,%f,%f\n */

  /* TODO: Read the potentiometer voltage */
  //int32_t enc =  motor_encoder_getValue();
  
    /* TODO: Print the sample time and potentiometer voltage to the serial terminal in the format [time],[voltage] */
  //printf("%f,%ld\n", logCount*0.005, enc);
    /* TODO: Increment log count */
  //logCount++;
  observer_set_y();
  observer_update();
  //observer_get_theta();
  //observer_get_ptheta();
  // printf("%f,%f,%f,%f\n",logCount*0.005,observer_get_theta(),observer_get_ptheta(),get_motor_revs()); 
  printf("%f,%f,%f,%f\n",logCount*0.005,observer_get_theta(),observer_get_dtheta(),get_motor_revs()); 

    
   //printf("%f,%f,%f,%ld\n",logCount*0.005,get_acc_angle(),get_gyroZ(),enc); 

   /* TODO: Increment log count */
   logCount++;

   //* TODO: Stop logging once 5 seconds is reached */
   if(logCount == 1000){
        logging_stop();
   }
}