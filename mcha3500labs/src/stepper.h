#ifndef STEPPER_H
#define STEPPER_H

void stepper_motor_PWM_init(void);
void test_stepper_motor(void);
void stepper_init(void);
void set_motor_revs(float input);
float get_motor_revs(void);

#endif