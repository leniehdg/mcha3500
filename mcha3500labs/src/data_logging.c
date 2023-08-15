#include <inttypes.h> // For PRIxx and SCNxx macros
#include "stm32f4xx_hal.h" // to import UNUSED() macro
#include "cmsis_os.h" // Include CMSIS-RTOS header

// Modules that provide commands
#include "pendulum.h"

/* Variable declarations */
uint16_t logCount;

osTimerId_t timerHandle; // Timer handle

/* Function declarations */
static void log_pendulum(void *argument);



/* Function defintions */
static void log_pendulum(void *argument)
{
    /* TODO: Supress compiler warnings for unused arguments */
    UNUSED(argument);

    /* TODO: Read the potentiometer voltage */
    float voltage = pendulum_read_voltage();

    /* TODO: Print the sample time and potentiometer voltage to the serial terminal in the format [time],[
    voltage] */
    printf("[%d ms], [%f V]\n", osKernelGetTickCount(), voltage);

    /* TODO: Increment log count */
    logCount++;

    /* TODO: Stop logging once 2 seconds is reached (Complete this once you have created the stop function
    in the next step) */
    if (logCount >= 400) { // Assuming 5 ms interval, 400 intervals = 2000 ms (2 seconds)
        pend_logging_stop();
    }
}   


void logging_init(void)
{
    /* TODO: Initialize any necessary peripherals and resources */
    /* Create a one-shot timer with 5ms interval (200 Hz) */
    const osTimerAttr_t timerAttr = {
        .name = "DataLoggingTimer"
    };
    timerHandle = osTimerNew(log_pendulum, osTimerOnce, NULL, &timerAttr);
    
    // timer starts after initialisation
    pend_logging_start();
}

void pend_logging_start(void)
{
    /* Reset the logCount variable */
    logCount = 0;

    /* Start the data logging timer */
    osStatus_t status = osTimerStart(timerHandle, 5); // 5 ms interval
    /* TODO: Check status and handle any errors */
}

void pend_logging_stop(void)
{
    /* Stop the data logging timer */
    osStatus_t status = osTimerStop(timerHandle);
    /* TODO: Check status and handle any errors */
}

void logging_timer_callback(void *arg)
{
    /* This function is called when the timer interval elapses */
    logCount++;
    /* Your data logging code here */

    /* Restart the timer for the next interval */
    osTimerStart(timerHandle, 5); // 5 ms interval
}
