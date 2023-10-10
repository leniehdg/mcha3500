#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
// #include "stm32f4xx_hal_pwr_ex.h"
#include "uart.h"
#include "stepper.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
 #endif

static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim2;
static TIM_OC_InitTypeDef sConfig;
static TIM_OC_InitTypeDef sConfig2;
float x1 = 0;
float x2 = 0;
float p = 0;
float p2 = 0;


void stepper_motor_PWM_init(void)
{
	
	__TIM3_CLK_ENABLE();
	__TIM2_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	
	HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 71;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 10000
	;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&htim3);
	
	sConfig.OCMode = TIM_OCMODE_PWM1;
	sConfig.Pulse = 0;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_OC_ConfigChannel(&htim3, &sConfig, TIM_CHANNEL_4);
	
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	HAL_TIM_OC_Start(&htim3,TIM_CHANNEL_4);
	
	GPIO_InitTypeDef GPIO_InitStruct_In1;
	GPIO_InitStruct_In1.Pin = GPIO_PIN_0;
	GPIO_InitStruct_In1.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct_In1.Pull = GPIO_NOPULL;
	GPIO_InitStruct_In1.Speed = GPIO_SPEED_FREQ_HIGH;
		
	HAL_GPIO_Init(GPIOA,&GPIO_InitStruct_In1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	
	
	// motor 2 initialisation ///////////////////////////////////////////////////////////////////
	GPIO_InitTypeDef GPIO_InitStruct2;
	GPIO_InitStruct2.Pin = GPIO_PIN_7;
	GPIO_InitStruct2.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct2.Pull = GPIO_NOPULL;
	GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct2.Alternate = GPIO_AF1_TIM2;
	
	HAL_GPIO_Init(GPIOA,&GPIO_InitStruct2);
	
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 71;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 10000
	;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&htim2);
	
	sConfig2.OCMode = TIM_OCMODE_PWM1;
	sConfig2.Pulse = 0;
	sConfig2.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig2.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_OC_ConfigChannel(&htim2, &sConfig2, TIM_CHANNEL_4);
	
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_4);
	
	GPIO_InitTypeDef GPIO_InitStruct_In2;
	GPIO_InitStruct_In2.Pin = GPIO_PIN_1;
	GPIO_InitStruct_In2.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct_In2.Pull = GPIO_NOPULL;
	GPIO_InitStruct_In2.Speed = GPIO_SPEED_FREQ_HIGH;
		
	HAL_GPIO_Init(GPIOA,&GPIO_InitStruct_In2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	
	
}

void set_Vin1(float x1)
{	
	if (x1 == 0)
	{
		
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	}
	
	else if (x1 > 0)
	{
		p = 10000/((1600/(2*M_PI))*x1);
		__HAL_TIM_SET_PRESCALER(&htim3, p+1);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 5000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	}
	
	else if (x1 < 0)
	{
		p = 10000/((1600/(2*M_PI))*-x1);
		__HAL_TIM_SET_PRESCALER(&htim3, p+1);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 5000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	}
	
}

void set_Vin2(float x2)
{
	
	if (x2 == 0)
	{
		
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	}
	
	else if (x2 > 0)
	{
		p2 = 10000/((1600/(2*M_PI))*x2);
		__HAL_TIM_SET_PRESCALER(&htim2, p2+1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 5000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	}
	
	else if (x2 < 0)
	{
		p2 = 10000/((1600/(2*M_PI))*-x2);
		__HAL_TIM_SET_PRESCALER(&htim2, p2+1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 5000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	}	
}


void test_stepper_motor(void)
{
    while(1)
	{
	set_Vin1(1.0);
    set_Vin2(1.0);
	}
}