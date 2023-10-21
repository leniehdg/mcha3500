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

  if (logCount < 200)
  {

    observer_update();
    logCount++;
    
  }

  else
  {  

    /*      OBSERVER       */
    // 1. Get states
    observer_update();
    float dtheta = observer_get_dtheta();
    float theta = observer_get_theta();
    // printf("theta (deg): %f, dtheta: %f\n",theta*180/3.1415, dtheta);
    float ptheta = observer_get_ptheta(dtheta);
    // printf("ptheta: %f\n",ptheta);


    printf("\n \n \nOnto Controller yewwww \n \n \n");

    // /*      CONTROLLER     */
    // // 2. Send states
    // ctrl_set_x_int();   
    // // 3. Get control
    // ctrl_update();
    // printf("%f,%f\n",ctrl_get_dtheta(),ctrl_get_dPtheta());
    // float wiggle = getControl()*4;
    // printf("Control output %f\n", wiggle);

    // /*      MAGIC         */
    // // 4. Do control
    // set_motor_revs(wiggle);

  }
}
 