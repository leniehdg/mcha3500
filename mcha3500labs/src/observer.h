#ifndef OBSERVER_H
#define OBSERVER_H
/* Add function prototypes here */
void observer_init(void);

void observer_get_xh(void);
float observer_get_theta(void);
float observer_get_dtheta(void);
float observer_get_ptheta(void);

void observer_set_y1(float y1);
void observer_set_y2(float y2);
void observer_set_y(void);

void observer_set_u(float u);


void observer_update(void);
#endif