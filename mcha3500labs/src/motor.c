#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "motor.h"

static TIM_HandleTypeDef htim3;

void motor_PWM_init(void)
{
    // Enable TIM3 clock
    __HAL_RCC_TIM3_CLK_ENABLE();

    // Enable GPIOA clock
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    // Initialise PA6 as alternate function mode (AF2 - TIM3)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Initialize Timer 3
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 10000) - 1; // Assuming SystemCoreClock is running at 168MHz
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 100 - 1; // To get 10kHz frequency
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    // Configure Timer 3, channel 1
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    // Calculate the initial duty cycle value for 25%
    uint32_t initialDutyCycle = (htim3.Init.Period + 1) / 4;
    sConfigOC.Pulse = initialDutyCycle; // Set the initial duty cycle to 25%
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

    // Start Timer 3, channel 1 PWM
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

