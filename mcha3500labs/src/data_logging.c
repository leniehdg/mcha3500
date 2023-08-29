#include <stdint.h> 
#include "stm32f4xx_hal.h" // to import UNUSED() macro
#include "cmsis_os2.h" // Include CMSIS-RTOS header

// Modules that provide commands
#include "pendulum.h"
#include "data_logging.h"

/* Variable declarations */
uint16_t logCount;
osTimerId_t timerHandle; // Timer handle
static void (*log_function) (void);

/* Function declarations */
static void log_pointer(void *argument);



/* Function defintions */


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


void logging_stop(void)
{
    /* Stop the data logging timer */
    osStatus_t status = osTimerStop(timerHandle);
    
}

/*      PENDULUM        */


static void log_pendulum(void)
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

void pend_logging_start(void)
{
    /* Reset the logCount variable */
    logCount = 0;

    /* function pointer to pend logging func */
    log_function = &log_pendulum;

    /* Start the data logging timer */
    osStatus_t status = osTimerStart(timerHandle, 5); // 5 ms interval
    
}