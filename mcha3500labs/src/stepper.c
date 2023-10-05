#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "encoder.h"
#include "stepper.h"

static GPIO_InitTypeDef  GPIO_InitStructure1;
static TIM_HandleTypeDef htim1;
static TIM_OC_InitTypeDef  _sConfigPWM1;

static GPIO_InitTypeDef  GPIO_InitStructure2;
static TIM_HandleTypeDef htim2;
static TIM_OC_InitTypeDef  _sConfigPWM2;

static GPIO_InitTypeDef  GPIO_InitStructure3;
static GPIO_InitTypeDef  GPIO_InitStructure4;

float velocity;
float K;
float prescaler;
float revs;

// float velocity;
// float K;
// float prescaler;

void stepper_PWM_init(void)
{
    // *** TOP MOTOR DRIVER (driver2    STEP2 = PA7 DIR2 = PA1) 
    // *** BOTTOM DRIVER    (driver1    STEP1 = PA6 DIR1 = PA0) 


    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    //////////////////////////// driver1 ////////////////////////////
    // STEP1 - PA6
    GPIO_InitStructure1.Pin = GPIO_PIN_6;                   
    GPIO_InitStructure1.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure1.Pull = GPIO_NOPULL;
    GPIO_InitStructure1.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure1.Alternate = GPIO_AF1_TIM1;      // TIM1
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure1);

    // TIMER 1
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 400;    // approx 10 kHz
    htim1.Init.ClockDivision = 1;
    HAL_TIM_PWM_Init(&htim1);

    // PWM1
    _sConfigPWM1.OCMode = TIM_OCMODE_PWM1;
    _sConfigPWM1.Pulse = 0;
    _sConfigPWM1.OCPolarity = TIM_OCPOLARITY_HIGH;
    _sConfigPWM1.OCFastMode = TIM_OCFAST_DISABLE;

    // TIM1, PWM1, CHANNEL 2
    HAL_TIM_PWM_ConfigChannel(&htim1, &_sConfigPWM1, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 200);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    
    // DIR1 - PA0
    GPIO_InitStructure3.Pin = GPIO_PIN_0;                      
    GPIO_InitStructure3.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure3.Pull = GPIO_NOPULL;
    GPIO_InitStructure3.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure3);


    //////////////////////////// driver2 ////////////////////////////
    // STEP1 - PA6
    GPIO_InitStructure2.Pin = GPIO_PIN_7;                              
    GPIO_InitStructure2.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure2.Pull = GPIO_NOPULL;
    GPIO_InitStructure2.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure2.Alternate = GPIO_AF2_TIM3;      // TIM3
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure2);

    // TIMER 3
    htim2.Instance = TIM3;
    htim2.Init.Prescaler = 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 400;
    htim2.Init.ClockDivision = 1;
    HAL_TIM_PWM_Init(&htim2);

    // PWM2
    _sConfigPWM2.OCMode = TIM_OCMODE_PWM1;  // receives same PWM as driver 1
    _sConfigPWM2.Pulse = 0;
    _sConfigPWM2.OCPolarity = TIM_OCPOLARITY_HIGH;
    _sConfigPWM2.OCFastMode = TIM_OCFAST_DISABLE;

    // TIM3, PWM1, CHANNEL 3
    HAL_TIM_PWM_ConfigChannel(&htim2, &_sConfigPWM2, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 200);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    // DIR2 - PA1
    GPIO_InitStructure4.Pin = GPIO_PIN_1;                      
    GPIO_InitStructure4.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure4.Pull = GPIO_NOPULL;
    GPIO_InitStructure4.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure4);
}

 void stepper_init(void)
 {
    //int motor = 2;
    // revs = -2;
    // printf("REVS %f\n",revs);
    // set_motor_revs(revs);
 }


void set_motor_revs(float input)
{
    printf("REVS %f\n",input);
    revs = input;
     input = input / 6.2831853071796; //convert from rads to revs   

     // IF CLOSE TO ZERO (positive) --> sets DIR to move the motor clockwise (REsetting two GPIO pins).
     if (input <=0.05 && input > 0)
     {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);      // DIR2
        //printf("Pin1");
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);      // DIR1
        velocity = 0.1*360*3.14159265359/180;
        K = (200*2*3.14159265359)*32; //200 Steps per rev
        prescaler = (1e7)/(velocity*K);
         __HAL_TIM_SET_PRESCALER(&htim1, prescaler);               // driver1
         __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
         //printf("REVS %f\n",input);
     }

    // IF CLOSE TO ZERO (negative) --> sets DIR to move the motor counterclockwise (SETTING two GPIO pins). 
     if (input <=0 && input >= -0.05)
     {
        velocity = 0.1*360*3.14159265359/180;
        K = (200*2*3.14159265359)*32; //200 Steps per rev
        prescaler = (1e7)/(velocity*K);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);     // DIR2
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);     // DIR1
         __HAL_TIM_SET_PRESCALER(&htim1, prescaler);            // driver1
         __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
         //printf("REVS %f\n",input);
     }  
     else if (input > 0.05) //if (input < 0.05 && input > -0.05)
     {
     velocity = input*360*3.14159265359/180;    // input * 360 degrees, converted to rad
     K = (200*2*3.14159265359)*32; //200 Steps per rev
     prescaler = (1e7)/(velocity*K);

     //printf(" %f Prescalar \n", prescaler);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
     //printf("Pin1");
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
     //printf("Pin2");  
     __HAL_TIM_SET_PRESCALER(&htim1, prescaler);    // driver1
     __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
     //printf("REVS %f\n",input);
     }  
     else if (input < -0.05)        //// reverse?
     {
    input = -input;
    velocity = input*360*3.14159265359/180;
    K = (200*2*3.14159265359)*32; //200 Steps per rev
    prescaler = (1e7)/(velocity*K);
    
    //printf(" %f Prescalar \n", prescaler);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
    
    __HAL_TIM_SET_PRESCALER(&htim1, prescaler);     // driver1
    __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
    //printf("REVS %f\n",input);
    }
    
    //printf("REVS %f\n",revs);
}

float get_motor_revs(void)
{
    return(revs);
}

// Sys ID. Swinging pendulum, with encoder
// Karman filter - Print MPU and encoder
// Observer and controller - lectorial 8 shows how - needs a little modification