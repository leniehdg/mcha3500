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

uint16_t logCount;

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
    osTimerStart(controlLoop,5);
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
    /*   IMU    */
    IMU_read();

    /*  Observer   */
    observer_set_y();
    observer_update();

    /*  Controller  */
    ctrl_set_x_int();
    ctrl_update();
    // ctrl_get_dPtheta();
    // ctrl_get_dtheta();
    printf("%f,%f\n",ctrl_get_dtheta(),ctrl_get_dPtheta());

    /*  Magic   */
    float wiggle = getControl()*0.5;
    //printf("CONTROL %f\n",rubbish);
    set_motor_revs(wiggle);
  }
}
 