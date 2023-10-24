#ifndef OBSERVER_H
#define OBSERVER_H

void observer_init(void);
float observer_update(void);
float observer_get_theta(void);
float observer_get_dtheta(void);
float observer_get_ptheta(float);


#endif