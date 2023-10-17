#include "stepper.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

static TIM_HandleTypeDef htim3;
static TIM_OC_InitTypeDef sConfigPWM1;
static TIM_HandleTypeDef htim4;
static TIM_OC_InitTypeDef sConfigPWM2;

float revs;
float velocity;

// Define GPIO Port and Pins for Motor 1 and Motor 2
#define MOTOR1_STP_PIN GPIO_PIN_6
#define MOTOR1_DIR_PIN GPIO_PIN_0
#define MOTOR1_STP_PORT GPIOA
#define MOTOR1_DIR_PORT GPIOA
#define MOTOR1_FORWARD GPIO_PIN_SET
#define MOTOR1_BACKWARD GPIO_PIN_RESET

#define MOTOR2_STP_PIN GPIO_PIN_4
#define MOTOR2_DIR_PIN GPIO_PIN_1
#define MOTOR2_STP_PORT GPIOB
#define MOTOR2_DIR_PORT GPIOA
#define MOTOR2_FORWARD GPIO_PIN_RESET
#define MOTOR2_BACKWARD GPIO_PIN_SET

// Define GPIO Port and Pins for MS1, MS2, and MS3
#define MS1_PIN GPIO_PIN_13
#define MS2_PIN GPIO_PIN_14
#define MS3_PIN GPIO_PIN_15
#define MS_PORT GPIOB

//------------------------------------------INIT FOR MOTOR PINS------------------------------------------//

void stepper_motor_PWM_init(void) {
    // Initialize TIM3 for PWM on PA6
    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 5000; // Adjust this value for the desired PWM frequency
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    sConfigPWM1.OCMode = TIM_OCMODE_PWM1;
    sConfigPWM1.Pulse = 0; // Adjust this value for the desired duty cycle
    sConfigPWM1.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigPWM1.OCFastMode = TIM_OCFAST_ENABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigPWM1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    // Initialize TIM4 for PWM on PB4
    __HAL_RCC_TIM4_CLK_ENABLE();
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 5000; // Adjust this value for the desired PWM frequency
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim4);

    sConfigPWM2.OCMode = TIM_OCMODE_PWM1;
    sConfigPWM2.Pulse = 0; // Adjust this value for the desired duty cycle
    sConfigPWM2.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigPWM2.OCFastMode = TIM_OCFAST_ENABLE;
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigPWM2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    // GPIO configuration for PA6
    GPIO_InitTypeDef GPIO_InitStruct1;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct1.Pin = MOTOR1_STP_PIN;
    GPIO_InitStruct1.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct1.Pull = GPIO_NOPULL;
    GPIO_InitStruct1.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct1.Alternate = GPIO_AF2_TIM3; // For PA6
    HAL_GPIO_Init(MOTOR1_STP_PORT, &GPIO_InitStruct1);

    // GPIO configuration PB4
    GPIO_InitTypeDef GPIO_InitStruct2;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct2.Pin = MOTOR2_STP_PIN;
    GPIO_InitStruct2.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct2.Pull = GPIO_NOPULL;
    GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct2.Alternate = GPIO_AF2_TIM4; // For PB4
    HAL_GPIO_Init(MOTOR2_STP_PORT, &GPIO_InitStruct2);

    // 50% duty cycle
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2500);
    // Start PWM
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  
    
    
    // Motor 1 direction signal initialisation
    GPIO_InitTypeDef GPIO_InitStruc3;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruc3.Pin = MOTOR1_DIR_PIN;
    GPIO_InitStruc3.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruc3.Pull = GPIO_NOPULL;
    GPIO_InitStruc3.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR1_DIR_PORT, &GPIO_InitStruc3);
    HAL_GPIO_WritePin(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, GPIO_PIN_SET);

    // Motor 2 direction signal initialisation
    GPIO_InitTypeDef GPIO_InitStruc4;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruc4.Pin = MOTOR2_DIR_PIN;
    GPIO_InitStruc4.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruc4.Pull = GPIO_NOPULL;
    GPIO_InitStruc4.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR2_DIR_PORT, &GPIO_InitStruc4);
    HAL_GPIO_WritePin(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, GPIO_PIN_SET);

    // GPIO configuration for microstepping
    GPIO_InitTypeDef GPIO_InitStruct3;
    // Enable GPIO Port Clock
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // Configure MS1, MS2, and MS3 pins as general-purpose output pins
    GPIO_InitStruct3.Pin = MS1_PIN | MS2_PIN | MS3_PIN;
    GPIO_InitStruct3.Mode = GPIO_MODE_OUTPUT_PP;  // Output in push-pull mode
    GPIO_InitStruct3.Pull = GPIO_NOPULL;          // No pull-up/pull-down
    GPIO_InitStruct3.Speed = GPIO_SPEED_FREQ_LOW; // Low-speed output
    HAL_GPIO_Init(MS_PORT, &GPIO_InitStruct3);
    // Set initial microstepping configuration
    HAL_GPIO_WritePin(MS_PORT, MS1_PIN, GPIO_PIN_RESET); // MS1 = 0
    HAL_GPIO_WritePin(MS_PORT, MS2_PIN, GPIO_PIN_RESET); // MS2 = 0
    HAL_GPIO_WritePin(MS_PORT, MS3_PIN, GPIO_PIN_RESET); // MS3 = 0
}

//------------------------------------------MICROSTEPPING DEFINE FUNCTIONS------------------------------------------//
/*
    For driver DRV8825: MS1 MS2 MS3 Result
        0 0 0 Full
        1 0 0 Half
        0 1 0 Quarter
        1 1 0 Eighth
        0 0 1 Sixteenth
        1 0 1 1/32
        0 1 1 1/32
        1 1 1 1/32
    For motor 1:
        STP: A6
        DIR: A0
    For motor 2:
        STP: B4
        DIR: A1
    For both motors:
        MS1: B13
        MS2: B14
        MS3: B15
*/

void Microstepping_SetFullStep(void) {
    HAL_GPIO_WritePin(MS_PORT, MS1_PIN, GPIO_PIN_RESET); // MS1 = 0
    HAL_GPIO_WritePin(MS_PORT, MS2_PIN, GPIO_PIN_RESET); // MS2 = 0
    HAL_GPIO_WritePin(MS_PORT, MS3_PIN, GPIO_PIN_RESET); // MS3 = 0
}

void Microstepping_SetHalfStep(void) {
    HAL_GPIO_WritePin(MS_PORT, MS1_PIN, GPIO_PIN_SET);   // MS1 = 1
    HAL_GPIO_WritePin(MS_PORT, MS2_PIN, GPIO_PIN_RESET); // MS2 = 0
    HAL_GPIO_WritePin(MS_PORT, MS3_PIN, GPIO_PIN_RESET); // MS3 = 0
}

void Microstepping_SetQuarterStep(void) {
    HAL_GPIO_WritePin(MS_PORT, MS1_PIN, GPIO_PIN_RESET); // MS1 = 0
    HAL_GPIO_WritePin(MS_PORT, MS2_PIN, GPIO_PIN_SET);   // MS2 = 1
    HAL_GPIO_WritePin(MS_PORT, MS3_PIN, GPIO_PIN_RESET); // MS3 = 0
}

void Microstepping_SetEighthStep(void) {
    HAL_GPIO_WritePin(MS_PORT, MS1_PIN, GPIO_PIN_SET);   // MS1 = 1
    HAL_GPIO_WritePin(MS_PORT, MS2_PIN, GPIO_PIN_SET);   // MS2 = 1
    HAL_GPIO_WritePin(MS_PORT, MS3_PIN, GPIO_PIN_RESET); // MS3 = 0
}

void Microstepping_SetSixteenthStep(void) {
    HAL_GPIO_WritePin(MS_PORT, MS1_PIN, GPIO_PIN_RESET); // MS1 = 0
    HAL_GPIO_WritePin(MS_PORT, MS2_PIN, GPIO_PIN_RESET); // MS2 = 0
    HAL_GPIO_WritePin(MS_PORT, MS3_PIN, GPIO_PIN_SET);   // MS3 = 1
}

void Microstepping_SetThirtySecondStep(void) {
    HAL_GPIO_WritePin(MS_PORT, MS1_PIN, GPIO_PIN_SET);   // MS1 = 1
    HAL_GPIO_WritePin(MS_PORT, MS2_PIN, GPIO_PIN_RESET); // MS2 = 0
    HAL_GPIO_WritePin(MS_PORT, MS3_PIN, GPIO_PIN_SET);   // MS3 = 1
}

//------------------------------------------TEST FUNCTION------------------------------------------//

void test_stepper_motor(void)
{
    printf("Starting test I thinks\n");
    // Number of steps for the motors to take
    int num_steps = 100;

    // Initialise the motors and PWM
    stepper_motor_PWM_init();
    Microstepping_SetThirtySecondStep();

    // Initialise the direction for both motors
    HAL_GPIO_WritePin(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, MOTOR1_FORWARD); // Set direction for motor 1 to forward
    HAL_GPIO_WritePin(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, MOTOR2_FORWARD); // Set direction for motor 2 to forward

    // Loop to generate steps
    for (int i = 0; i <= num_steps; i++)
    {
        // Generate dummy prescaler
        float prescaler = 40.0;
        //printf("Single step\n");

        // Set the prescaler value for TIM3 and TIM4
        __HAL_TIM_SET_PRESCALER(&htim3, prescaler+1);
        __HAL_TIM_SET_PRESCALER(&htim4, prescaler+1);
        //printf("Prescaler %f\n", prescaler);

        // Set the compare value for TIM3 and TIM4, channel 1

    }

    // // Turn off both motors
    // __HAL_TIM_SET_PRESCALER(&htim3, 0);
    // __HAL_TIM_SET_PRESCALER(&htim4, 0);
    // // Set the compare value for TIM3 and TIM4, channel 1
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    // HAL_GPIO_WritePin(MOTOR1_STP_PORT, MOTOR1_STP_PIN, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(MOTOR2_STP_PORT, MOTOR2_STP_PIN, GPIO_PIN_RESET);
}

//------------------------------------------MOTOR OPERATION FUNCTION------------------------------------------//