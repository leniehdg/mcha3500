#ifndef MOTOR_H
#define MOTOR_H

/* Add function prototypes here */
void motor_PWM_init(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
int32_t motor_encoder_getValue(void);
int32_t motor_encoder_getValue(void);
void motor_encoder_init(void);

#endif