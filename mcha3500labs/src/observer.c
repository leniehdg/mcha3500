#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "heartbeat_task.h"
#include "data_logging.h"
#include "pendulum.h"
#include "IMU.h"
#include "observer.h"
#include "arm_math.h"
#include <stdint.h>

#define HEARTBEATPERIOD 1000

static float32_t Kb_f32[6] =        
{
	// CHANGE FROM KALMAN_FILTER SCRIPT: optimal kalman gain, called Kk in MATLAB script
	0.1399,    1.2619,
    0.0353,    0.0854,
   -0.1190,   -0.2842,
};


static float32_t yi_f32[2] =
{
	0,   
	0
};

static float32_t C_f32[6] =
{
	0.0,   1.0, 0.0,
    1.0,   0.0, 1.0,
};


static float32_t xhm_f32[3] =
{
	0.0,  
    0.0, 
    0.0,  
};

static float32_t xhp_f32[3] =
{
	0.0,  
    0.0, 
    0.0,  
};

static float32_t Ad_f32[9] =
{
	1.0,    0.0,    0.0,
    0.005,  1.0,    0.0,
    0.0,    0.0,    1.0,
};


static float32_t yhat_f32[2] = // This is C * Xhm
{
	0.0,  
	0.0,
};


static float32_t ye_f32[2] = // Yi- Yhat
{
	0.0,  
	0.0,
};


static float32_t Kbye_f32[3] = // Kb*ye
{
	0.0,  
	0.0,
	0.0,
};

arm_matrix_instance_f32 Kb={3,2,(float32_t *)Kb_f32};
arm_matrix_instance_f32 yi={2,1,(float32_t *)yi_f32};
arm_matrix_instance_f32 C={2,3,(float32_t *)C_f32};
arm_matrix_instance_f32 xhm={3,1,(float32_t *)xhm_f32};
arm_matrix_instance_f32 xhp={3,1,(float32_t *)xhp_f32};
arm_matrix_instance_f32 Ad={3,3,(float32_t *)Ad_f32};
arm_matrix_instance_f32 yhat={2,1,(float32_t *)yhat_f32};
arm_matrix_instance_f32 ye={2,1,(float32_t *)ye_f32};
arm_matrix_instance_f32 Kbye={3,1,(float32_t *)Kbye_f32};

void observer_init(void)
{
	arm_mat_init_f32(&Kb,3,2,(float32_t *)Kb_f32);
	arm_mat_init_f32(&yi,2,1,(float32_t *)yi_f32);
	arm_mat_init_f32(&C,2,3,(float32_t *)C_f32);
	arm_mat_init_f32(&xhm,3,1,(float32_t *)xhm_f32);
	arm_mat_init_f32(&xhp,3,1,(float32_t *)xhp_f32);
	arm_mat_init_f32(&Ad,3,3,(float32_t *)Ad_f32);
	arm_mat_init_f32(&yhat,2,1,(float32_t *)yhat_f32);
	arm_mat_init_f32(&ye,2,1,(float32_t *)ye_f32);
	arm_mat_init_f32(&Kbye,3,1,(float32_t *)Kbye_f32);
}


void observer_update(float y_measure, float y_measure2)
{
	yi_f32[0] = y_measure;
	yi_f32[1] = y_measure2;

	// Correction xp = xm + Kb*(yi-C*xm);
	arm_mat_mult_f32(&C,&xhm,&yhat);
	arm_mat_sub_f32(&yi,&yhat,&ye);
	arm_mat_mult_f32(&Kb,&ye,&Kbye);
	arm_mat_add_f32(&xhm,&Kbye,&xhp);

	// Prediction xm = Ad*xp;
	arm_mat_mult_f32(&Ad,&xhp,&xhm);
}


/* xhm = predicted state after time step  */

float get_obs_x1(void)
{
	return xhm_f32[0];
}

float get_obs_x2(void)
{
	return xhm_f32[1];
}
