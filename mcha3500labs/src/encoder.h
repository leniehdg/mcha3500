#ifndef ENCODER_H
#define ENCODER_H

/* Add function prototypes here */
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
int32_t motor_encoder_getValue(void);
void motor_encoder_init(void);

#endif