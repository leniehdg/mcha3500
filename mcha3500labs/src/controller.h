#ifndef CONTROLLER_H
#define CONTROLLER_H

/* Add function prototypes here */
void ctrl_init(void);
void ctrl_set_x1(float x1);
void ctrl_set_x2(float x2);
void ctrl_set_x3(float x3);
void ctrl_set_x4(float x4);
float getControl(void);
void ctrl_update(void);

#endif
