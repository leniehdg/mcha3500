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
    /* TODO: Enable GPIOA clock */
    /* TODO: Enable TIM3 clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* TODO: Initialise PA9 with:
    - Pin 9
    - Alternate function push-pull mode
    - No pull
    - High frequency
    - Alternate function 1 - Timer 1*/
    
    GPIO_InitStructure1.Pin = GPIO_PIN_9;
    GPIO_InitStructure1.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure1.Pull = GPIO_NOPULL;
    GPIO_InitStructure1.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure1.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure1);

    /* TODO: Initialise timer 3 with:
    - Instance TIM1
    - Prescaler of 1
    - Counter mode up
    - Timer period to generate a 10kHz signal
    - Clock division of 0 */
        htim1.Instance = TIM1;
        htim1.Init.Prescaler = 1;
        htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim1.Init.Period = 400;
        htim1.Init.ClockDivision = 1;
        HAL_TIM_PWM_Init(&htim1);

    /* TODO: Configure timer 3, channel 1 with:
    - Output compare mode PWM
    - Pulse = 0
    - OC polarity high
    - Fast mode disabled */
        _sConfigPWM1.OCMode = TIM_OCMODE_PWM1;
        _sConfigPWM1.Pulse = 0;
        _sConfigPWM1.OCPolarity = TIM_OCPOLARITY_HIGH;
        _sConfigPWM1.OCFastMode = TIM_OCFAST_DISABLE;

    /* TODO: Set initial Timer 3, channel 1 compare value */
    HAL_TIM_PWM_ConfigChannel(&htim1, &_sConfigPWM1, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 200);
    /* TODO: Start Timer 3, channel 1 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    /* TODO: Initialise PA9 with:
    - Pin 9
    - Alternate function push-pull mode
    - No pull
    - High frequency
    - Alternate function 1 - Timer 1*/
    
    GPIO_InitStructure2.Pin = GPIO_PIN_0;
    GPIO_InitStructure2.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure2.Pull = GPIO_NOPULL;
    GPIO_InitStructure2.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure2.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure2);

    /* TODO: Initialise timer 3 with:
    - Instance TIM1
    - Prescaler of 1
    - Counter mode up
    - Timer period to generate a 10kHz signal
    - Clock division of 0 */
        htim2.Instance = TIM3;
        htim2.Init.Prescaler = 1;
        htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim2.Init.Period = 400;
        htim2.Init.ClockDivision = 1;
        HAL_TIM_PWM_Init(&htim2);

    /* TODO: Configure timer 3, channel 1 with:
    - Output compare mode PWM
    - Pulse = 0
    - OC polarity high
    - Fast mode disabled */
        _sConfigPWM2.OCMode = TIM_OCMODE_PWM1;
        _sConfigPWM2.Pulse = 0;
        _sConfigPWM2.OCPolarity = TIM_OCPOLARITY_HIGH;
        _sConfigPWM2.OCFastMode = TIM_OCFAST_DISABLE;

    /* TODO: Set initial Timer 3, channel 1 compare value */
    HAL_TIM_PWM_ConfigChannel(&htim2, &_sConfigPWM2, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 200);
    /* TODO: Start Timer 3, channel 1 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    GPIO_InitStructure3.Pin = GPIO_PIN_10;
    GPIO_InitStructure3.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure3.Pull = GPIO_NOPULL;
    GPIO_InitStructure3.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure3);

    GPIO_InitStructure4.Pin = GPIO_PIN_1;
    GPIO_InitStructure4.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure4.Pull = GPIO_NOPULL;
    GPIO_InitStructure4.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure4);
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
     if (input <=0.05 && input > 0)
     {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
        //printf("Pin1");
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        velocity = 0.1*360*3.14159265359/180;
        K = (200*2*3.14159265359)*32; //200 Steps per rev
        prescaler = (1e7)/(velocity*K);
         __HAL_TIM_SET_PRESCALER(&htim1, prescaler);
         __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
         //printf("REVS %f\n",input);
     }

     if (input <=0 && input >= -0.05)
     {
        velocity = 0.1*360*3.14159265359/180;
        K = (200*2*3.14159265359)*32; //200 Steps per rev
        prescaler = (1e7)/(velocity*K);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
         __HAL_TIM_SET_PRESCALER(&htim1, prescaler);
         __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
         //printf("REVS %f\n",input);
     }  
     else if (input > 0.05) //if (input < 0.05 && input > -0.05)
     {
     velocity = input*360*3.14159265359/180;
     K = (200*2*3.14159265359)*32; //200 Steps per rev
     prescaler = (1e7)/(velocity*K);

     //printf(" %f Prescalar \n", prescaler);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
     //printf("Pin1");
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
     //printf("Pin2");  
     __HAL_TIM_SET_PRESCALER(&htim1, prescaler);
     __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
     //printf("REVS %f\n",input);
     }  
     else if (input < -0.05)
     {
    input = -input;
    velocity = input*360*3.14159265359/180;
    K = (200*2*3.14159265359)*32; //200 Steps per rev
    prescaler = (1e7)/(velocity*K);
    
    //printf(" %f Prescalar \n", prescaler);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
    
    __HAL_TIM_SET_PRESCALER(&htim1, prescaler);
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