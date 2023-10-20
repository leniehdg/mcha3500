#ifndef OBSERVER_H
#define OBSERVER_H

void observer_init(void);
void observer_update(float y_measure, float y_measure2);
float observer_get_theta(void);
float observer_get_dtheta(void);
float observer_get_ptheta(void);
void observer_set_y(void);


#endif