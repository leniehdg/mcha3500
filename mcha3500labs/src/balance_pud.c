#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "uart.h"
#include "cmsis_os2.h"
#include "arm_math.h" /* Include STM32 DSP matrix libraries */
#include "controller.h"
#include "IMU.h"
#include "observer.h"
#include "stepper.h"
#include "balance_pud.h"


static osTimerId_t controlLoop;

float logCount;

static void (*ctrl_function)(void);

static void ctrl_pointer(void *argument);

static void balance_begin(void);



void balance_init(void)
{
    //for (int i=0;i<10000;i++)
    controlLoop = osTimerNew(ctrl_pointer, osTimerPeriodic, NULL, NULL);
  
    ctrl_start();
}


static void ctrl_pointer(void *argument)
{   
    UNUSED(argument); 

    /* Call function pointed to by log_function */
    (*ctrl_function)();
}


void ctrl_start(void)
{   
    // HAL_Delay(1);
    ctrl_function = &balance_begin;
    
    /* Reset the log counter */
    logCount = 0;

    /* Start data logging timer at 200Hz */
    osTimerStart(controlLoop,5);    // 5 milliseconds
}


static void balance_begin(void)
{

    /*      OBSERVER       */
    // 1. Get states
    // float accel_angle = 
    observer_update();
    float theta = observer_get_theta();
    float dtheta = observer_get_dtheta();
    // printf("theta (deg): %f, dtheta: %f\n",theta*180/3.1415, dtheta);
    float ptheta = observer_get_ptheta(dtheta);
    // printf("theta (deg): %f, ptheta: %f\n", theta*180/3.1415, ptheta);


    /*      CONTROLLER     */
    // 2. Send states
    ctrl_set_x1_int(theta);  
    ctrl_set_x2_int(ptheta);
    // 3. Get control
    ctrl_update();
    float wiggle = getControl();
    // wiggle = (wiggle - 51);     // restart offset
    printf("Control output: %f\n", wiggle);


    // Initial wait period: 2 seconds
    if (logCount < 400)
    {

        logCount++;
        
    }

    else
    {  
        /*      MAGIC         */
        // 4. Do control
        // set_motor_revs(wiggle);
        // float vel = get_motor_revs();
        // test_stepper_motor();       // 9.7 at 100, 4.87 at 50, 1 at 10, 0.5 at 5

    }
}
 