#ifndef STEPPER_H
#define STEPPER_H

void stepper_motor_PWM_init(void);
void test_stepper_motor(void);
void stepper_init(void);
void set_motor_revs(float);
float get_motor_revs(void);
void start_motor(void);
void stop_motor(void);
void Microstepping_SetFullStep(void);
void Microstepping_SetHalfStep(void);
void Microstepping_SetQuarterStep(void);
void Microstepping_SetEighthStep(void);
void Microstepping_SetSixteenthStep(void);
void Microstepping_SetThirtySecondStep(void);


#endif