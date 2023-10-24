#ifndef CONTROLLER_H
#define CONTROLLER_H

/* Add function prototypes here */
void ctrl_init(void);
void ctrl_set_x1_int(float x1);
void ctrl_set_x2_int(float x2);
float getControl(void);
void ctrl_update(void);


#endif
