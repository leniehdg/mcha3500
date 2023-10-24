#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "heartbeat_task.h"
#include "IMU.h"
#include "observer.h"
#include "stepper.h"
#include "arm_math.h"
#include <stdint.h>

#define HEARTBEATPERIOD 1000

// INITIALISE from MATLAB
static float32_t Kb_f32[6] =        // optimal kalman gain, last Kk value in Kalman_script.m
{
	0.1399,    1.2619,
    0.0353,    0.0854,
   -0.1190,   -0.2842,
};

// INITIALISE from MATLAB 
static float32_t yi_f32[2] =
{
	0,   
	0

	// tom's:
	// -0.3450,
	// -0.4560

	// // mine:
	//  0.2910,
    // -0.3670
};

// INITIALISE from MATLAB
static float32_t C_f32[6] =
{
	0.0,   1.0,   0.0,
    1.0,   0.0,   1.0,
};


static float32_t xhm_f32[3] =
{
	0.0,  
    0.0, 
    0.0,  
};

// INITIALISE from MATLAB 
static float32_t xhp_f32[3] =
{
	0.0,  
    0.0, 
    0.0,  

	// tom's:
	// -0.452101973764415,
	// -0.362912589510825,
	// -0.00371741272527977,

	// // mine:
	// -0.3911,
    // 0.2301,
    // 0.0260,
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


void observer_update()
{
	/*	Set y	*/

    yi_f32[0] = get_acc_angle();	
    yi_f32[1] = get_gyroY();

	// printf("IMU --> accel_angle: %f, vel: %f\n", yi_f32[0], yi_f32[1]);


    /* Kalman filter update steps	*/

    // 1. Calculate estimated measurement yhat = C * xhm
    arm_mat_mult_f32(&C, &xhm, &yhat);
    // 2. Calculate measurement error ye = yi - yhat
    arm_mat_sub_f32(&yi, &yhat, &ye);
    // 3. Calculate the Kalman gain correction Kbye = Kb * ye
    arm_mat_mult_f32(&Kb, &ye, &Kbye);
    // 4. Update the predicted state xhp = xhm + Kbye
    arm_mat_add_f32(&xhm, &Kbye, &xhp);
    // 5. Update the estimated state xhm = Ad * xhp
    arm_mat_mult_f32(&Ad, &xhp, &xhm);

    // printf("OBS --> theta = %f dtheta = %f\n", xhm_f32[0], xhm_f32[1]);

	// return yi_f32[0];	// send imu accel_angle

	/* 	
	
	PROGRESS TO THIS?? 	
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
    printf("dtheta = %f theta = %f\n", obs_xh_f32[1],obs_xh_f32[0]);

	*/
}


/* xhm = predicted state after time step  */

float observer_get_theta(void)	// x1
{
	return xhm_f32[1];
}

float observer_get_dtheta(void)	// x2
{
	return xhm_f32[0];
}

float observer_get_ptheta(float dtheta)
{
	// GET THESE FROM MATLAB CLASSBALANCINGROBOTFLOWLINEARISED
    float A = 46.9783;
    float B = 0.2900;

    float omega = get_motor_revs();		// velocity (rad/s)

	// ASSUMING DTHETA IS SECOND STATE (xhm_f32[1])
    float ptheta = (dtheta + omega*B) / A;   // dtheta/A + B + A*omega

    return ptheta;
}