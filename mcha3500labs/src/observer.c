#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "heartbeat_task.h"
#include "IMU.h"
#include "observer.h"
#include "stepper.h"
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


void observer_set_y(void)
{
    yi_f32[0] = get_acc_angle();
    yi_f32[1] = get_gyroY();
}


void observer_update()
{
	// yi_f32[0] = y_measure;	// get_acc_angle
	// yi_f32[1] = y_measure2;	// get_gyroZ
	// observer_set_y();
	
	// Correction xp = xm + Kb*(yi-C*xm);
	arm_mat_mult_f32(&C,&xhm,&yhat);
	arm_mat_sub_f32(&yi,&yhat,&ye);
	arm_mat_mult_f32(&Kb,&ye,&Kbye);
	arm_mat_add_f32(&xhm,&Kbye,&xhp);

	// Prediction xm = Ad*xp;
	arm_mat_mult_f32(&Ad,&xhp,&xhm);

	/* 	
	
	SHOULD IT BE THIS?? 	
	//Predict Mesurement
    arm_mat_mult_f32(&obs_Ad, &obs_xh, &obs_Adx);
    arm_mat_mult_f32(&obs_Bd, &obs_u, &obs_Bdu);
    arm_mat_add_f32(&obs_Adx, &obs_Bdu, &obs_AdxpBdu);
    arm_mat_mult_f32(&obs_C, &obs_AdxpBdu, &obs_yh);

    //Compute update
    arm_mat_sub_f32(&obs_y, &obs_yh, &obs_yt);
    //printf("%f,%f\n", obs_yt_f32[0],obs_yt_f32[1]);
    arm_mat_mult_f32(&obs_Kb, &obs_yt, &obs_Kyt);

    //Update prediction
    arm_mat_add_f32(&obs_AdxpBdu, &obs_Kyt, &obs_xh);
    //printf("dtheta = %f theta = %f\n", obs_xh_f32[1],obs_xh_f32[0]);

	*/
}


/* xhm = predicted state after time step  */

// IS DTHETA STATE 0 OR 1 ???

float observer_get_theta(void)	// x2??
{
	return xhm_f32[1];
}

float observer_get_dtheta(void)	// x1??
{
	return xhm_f32[0];
}

float observer_get_ptheta(void)
{
	// GET THESE FROM CLASSBALANCINGROBOTFLOWLINEARISED
    float A = 46.9783;
    float B = 0.2900;

	// ASSUMING DTHETA IS SECOND STATE (xhm_f32[1])
    float ptheta = (xhm_f32[1] + get_motor_revs()*B) / A;   // dtheta/A + B + A*omega
    //float ptheta = observer_get_dtheta()*0.04;
    printf("ptheta = %f dtheta = %f\n", ptheta, xhm_f32[1]);
    return ptheta;
}