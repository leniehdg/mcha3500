#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "encoder.h"


static TIM_HandleTypeDef htim3;
int32_t enc_count = 0;

void motor_PWM_init(void)
{
    /* TODO: Enable TIM3 clock */
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* TODO: Enable GPIOA clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* TODO: Initialise PA6 with:
    - Pin 6
    - Alternate function push-pull mode
    - No pull
    - High frequency
    - Alternate function 2 - Timer 3*/
    static GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TODO: Initialise timer 3 with:
    - Instance TIM3
    - Prescaler of 1
    - Counter mode up
    - Timer period to generate a 10kHz signal
    - Clock division of 0 */
    
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 10000;
    htim3.Init.ClockDivision = 0; //REPRESENTS NO CLOCK DIVISION
    HAL_TIM_PWM_Init(&htim3);

    /* TODO: Configure timer 3, channel 1 with:
    - Output compare mode PWM1
    - Pulse = 0
    - OC polarity high
    - Fast mode disabled */
    static TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

    /* TODO: Set initial Timer 3, channel 1 compare value */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 5000);

    /* TODO: Start Timer 3, channel 1 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

// Enable pin control
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // PA2 controls motor enable (enable)

// Motor direction control
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // PA1 controls motor direction (forward)
HAL_Delay(10000); // Delay for 10 seconds

// Disable the motor
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // Turn off motor (disable)

}

// static void drive_10_sec(void)
// {
//     //thread
//     //__HAL_TIM_SET_PRESCALER(&htim3, 2);   

//     // Additional code to drive forward for 10 seconds
//     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // PA1 controls motor direction (forward)
//     HAL_Delay(10000); // Delay for 10 seconds
//     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // Turn off motor
// }



void motor_encoder_init(void)
{
    /* TODO: Enable GPIOC clock */ 
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /* TODO: Initialise PC0, PC1 with:
    - Pin 0|1
    - Interrupt rising and falling edge
    - No pull
    - High frequency */
    static GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* TODO: Set priority of external interrupt lines 0,1 to 0x0f, 0x0f
    To find the IRQn_Type definition see "MCHA3500 Windows Toolchain\workspace\STM32Cube_F4_FW\Drivers\
    CMSIS\Device\ST\STM32F4xx\Include\stm32f446xx.h" */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0x0F, 0x0F);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0x0F, 0x0F);

    /* TODO: Enable external interrupt for lines 0, 1 */
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void EXTI0_IRQHandler(void)
{
/*TODO: Check if PC0 == PC1. Adjust encoder count accorrdingly*/
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1))
    {
        enc_count++; // Increment the encoder count if PC0 == PC1
    }
    else
    {
        enc_count--; // Decrement the encoder count if PC0 != PC1
    }

    /*TODO: Reset interrupt*/
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}



void EXTI1_IRQHandler(void)
{
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0))
    {
        enc_count--; // Increment the encoder count if PC0 == PC1
    }
    else
    {
        enc_count++; // Decrement the encoder count if PC0 != PC1
    } 
    
    /*TODO: Reset interrupt*/
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}



int32_t motor_encoder_getValue(void)
{
    return enc_count;
}
