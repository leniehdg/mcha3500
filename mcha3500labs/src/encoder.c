#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "encoder.h"


int32_t enc_count = 0;


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
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1))
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