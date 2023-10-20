#ifndef OBSERVER_H
#define OBSERVER_H

void observer_init(void);
void observer_update();
float observer_get_theta(void);
float observer_get_dtheta(void);
float observer_get_ptheta(void);
void observer_set_y(void);


#endif