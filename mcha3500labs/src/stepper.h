#ifndef STEPPER_H
#define STEPPER_H
/* Add function prototypes here */
void stepper_init(void);
void stepper_PWM_init(void);
void set_motor_revs(float);
float get_motor_revs(void);

#endif