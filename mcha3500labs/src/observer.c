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

static float32_t Kb_f32[6] =        // optimal kalman gain, called Kk in MATLAB script
{
	0.1304,    1.2434,
	0.0343,    0.0836,
   -0.1100,   -0.2654,
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

	/* 	SHOULD IT BE THIS?? 	*/
	// Prediction xm = Ad*xp + Bd*u + Kb*yt;
	arm_mat_mult_f32(&Ad, &xhp, &Adx);		/// not sure if xhp or xhm
    arm_mat_mult_f32(&obs_Bd, &obs_u, &Bdu);
    arm_mat_add_f32(&Adx, &Bdu, &Adx_Bdu);
    arm_mat_mult_f32(&obs_C, &obs_AdxpBdu, &obs_yh);
	arm_mat_add_f32(&Adx_Bdu, &xhp, &obs_xh);
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
