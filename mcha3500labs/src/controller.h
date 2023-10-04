#ifndef CONTROLLER_H
#define CONTROLLER_H
/* Add function prototypes here */
void ctrl_init(void);
void ctrl_update(void);


void ctrl_set_x1_int(float);
void ctrl_set_x2_int(float);
void ctrl_set_x_int(void);
//void ctrl_set_x3(float);
//void ctrl_set_x4(float);

float getControl(void);
float getdtheta(void);
float getdPtheta(void);

#endif