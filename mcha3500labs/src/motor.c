#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "motor.h"

static TIM_HandleTypeDef htim3;

// Define the encoder count variable
int32_t encoder_count = 0;

void motor_encoder_init(void)
{
  /* Enable GPIOC clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  static GPIO_InitTypeDef GPIO_InitStruct;

  /* Initialize PC0 and PC1 with:
   - Pin 0|1
   - Interrupt mode: rising and falling edge
   - No pull-up/pull-down
   - High-speed */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Set priority of external interrupt lines 0 and 1 to 0x0F */
  HAL_NVIC_SetPriority(6, 0x0F, 0); // 6 = EXTI0_IRQn
  HAL_NVIC_SetPriority(7, 0x0F, 0); // 7 = EXTI1_IRQn

  /* Enable external interrupt for lines 0 and 1 */
  HAL_NVIC_EnableIRQ(6);
  HAL_NVIC_EnableIRQ(7);
}



int32_t motor_encoder_getValue(void)
{
    return encoder_count;
}


void EXTI0_IRQHandler(void)
{
    /* TODO: Check if PC0 == PC1. Adjust encoder count accordingly. */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1))
    {
        // If PC0 and PC1 have the same state, increment the encoder count
        encoder_count++;
    }
    else
    {
        // If PC0 and PC1 have different states, decrement the encoder count
        encoder_count--;
    }

    /* Clear the EXTI interrupt pending bit for GPIO_PIN_0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI1_IRQHandler(void)
{
    /* TODO: Check if PC0 == PC1. Adjust encoder count accordingly. */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1))
    {
        // If PC0 and PC1 have the same state, increment the encoder count
        encoder_count--;
    }
    else
    {
        // If PC0 and PC1 have different states, decrement the encoder count
        encoder_count++;
    }

    /* Clear the EXTI interrupt pending bit for GPIO_PIN_1 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}


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
    htim3.Init.Prescaler = 1; // (HAL_RCC_GetPCLK1Freq() / 10000) - 1; // Assuming SystemCoreClock is running at 168MHz
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 100 - 1; // To get 10kHz frequency
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  // 0?
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

/*
    // Calculate the duty cycle value for 25% (initial = 0%)
    uint32_t dutyCycle = (htim3.Init.Period + 1) / 4;

    // Set the duty cycle for Timer 3, Channel 1
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, dutyCycle);

*/

}

