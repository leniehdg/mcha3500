#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "observer.h"
#include "arm_math.h"
#include "IMU.h"
#include "stepper.h"

#define OBS_N_INPUTS 1
#define OBS_N_STATES 3
#define OBS_N_OUTPUTS 2 //Angle, gyro velocity (veloicy + bias)

float theta;
float dtheta;

static float obs_Kb_f32[OBS_N_STATES*OBS_N_OUTPUTS] =
{
    0.0598,    1.1065,
    0.0244,    0.0546,
   -0.0598,   -0.1171,
};

static float obs_Ad_f32[OBS_N_STATES*OBS_N_STATES] =
{
    1.0, 0.0, 0.0,
    0.005, 1.0, 0.0,
    0.0, 0.0, 1.0,
};

static float obs_xh_f32[OBS_N_STATES] =
{
    0.0,
    0.0,
    0.0,
};

static float obs_Bd_f32[OBS_N_STATES*OBS_N_INPUTS] =
{
    0.0,
    0.0,
    0.0,
};

static float obs_u_f32[OBS_N_INPUTS] =
{
    0.0,
};

static float obs_Adx_f32[OBS_N_STATES] =
{
    0.0,
    0.0,
    0.0,
};

static float obs_Bdu_f32[OBS_N_STATES] =
{
    0.0,
    0.0,
    0.0,
};

static float obs_AdxpBdu_f32[OBS_N_STATES] =
{
    0.0,
    0.0,
    0.0,
};

static float obs_yh_f32[OBS_N_OUTPUTS] =
{
    0.0,
    0.0,
};

static float obs_C_f32[OBS_N_OUTPUTS*OBS_N_STATES] =
{
    0.0, 1.0, 0.0,
    1.0, 0.0, 1.0,
};

static float obs_y_f32[OBS_N_OUTPUTS] =
{
    0.0,
    0.0,
};

static float obs_yt_f32[OBS_N_OUTPUTS] =
{
    0.0,
    0.0,
};

static float obs_Kyt_f32[OBS_N_STATES] =
{
    0.0,
    0.0,
};

arm_matrix_instance_f32 obs_Kb = {OBS_N_STATES, OBS_N_OUTPUTS, (float32_t *)obs_Kb_f32};
arm_matrix_instance_f32 obs_Ad = {OBS_N_STATES, OBS_N_STATES, (float32_t *)obs_Ad_f32};
arm_matrix_instance_f32 obs_xh = {OBS_N_STATES, 1, (float32_t *)obs_xh_f32};
arm_matrix_instance_f32 obs_Bd = {OBS_N_STATES, OBS_N_INPUTS, (float32_t *)obs_Bd_f32};
arm_matrix_instance_f32 obs_u = {OBS_N_INPUTS, 1, (float32_t *)obs_u_f32};
arm_matrix_instance_f32 obs_Adx = {OBS_N_STATES, 1, (float32_t *)obs_Adx_f32};
arm_matrix_instance_f32 obs_Bdu = {OBS_N_STATES, 1, (float32_t *)obs_Bdu_f32};
arm_matrix_instance_f32 obs_AdxpBdu = {OBS_N_STATES, 1, (float32_t *)obs_AdxpBdu_f32};
arm_matrix_instance_f32 obs_yh = {OBS_N_OUTPUTS, 1, (float32_t *)obs_yh_f32};
arm_matrix_instance_f32 obs_C = {OBS_N_OUTPUTS,OBS_N_STATES, (float32_t *)obs_C_f32};
arm_matrix_instance_f32 obs_y = {OBS_N_OUTPUTS, 1, (float32_t *)obs_y_f32};
arm_matrix_instance_f32 obs_yt = {OBS_N_OUTPUTS, 1, (float32_t *)obs_yt_f32};
arm_matrix_instance_f32 obs_Kyt = {OBS_N_STATES, 1, (float32_t *)obs_Kyt_f32};






void observer_init(void)
{
    arm_mat_init_f32(&obs_Kb,OBS_N_STATES, OBS_N_OUTPUTS, (float32_t *)obs_Kb_f32);
    arm_mat_init_f32(&obs_Ad,OBS_N_STATES, OBS_N_STATES, (float32_t *)obs_Ad_f32);
    arm_mat_init_f32(&obs_xh,OBS_N_STATES, 1, (float32_t *)obs_xh_f32);
    arm_mat_init_f32(&obs_Bd,OBS_N_STATES, OBS_N_INPUTS, (float32_t *)obs_Bd_f32);
    arm_mat_init_f32(&obs_u,OBS_N_INPUTS, 1, (float32_t *)obs_u_f32);
    arm_mat_init_f32(&obs_Adx,OBS_N_STATES, 1, (float32_t *)obs_Adx_f32);
    arm_mat_init_f32(&obs_Bdu,OBS_N_STATES, 1, (float32_t *)obs_Bdu_f32);
    arm_mat_init_f32(&obs_AdxpBdu,OBS_N_STATES, 1, (float32_t *)obs_AdxpBdu_f32);
    arm_mat_init_f32(&obs_yh,OBS_N_OUTPUTS, 1, (float32_t *)obs_yh_f32);
    arm_mat_init_f32(&obs_C,OBS_N_OUTPUTS,OBS_N_STATES, (float32_t *)obs_C_f32);
    arm_mat_init_f32(&obs_y,OBS_N_OUTPUTS, 1, (float32_t *)obs_y_f32);
    arm_mat_init_f32(&obs_yt,OBS_N_OUTPUTS, 1, (float32_t *)obs_yt_f32);
    arm_mat_init_f32(&obs_Kyt,OBS_N_STATES, 1, (float32_t *)obs_Kyt_f32);
}

void observer_update(void)
{
    //observer_set_y();
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
}

void observer_set_y1(float y1)
{
    obs_y_f32[0] = y1;
    
}
void observer_set_y2(float y2)
{
    obs_y_f32[1] = y2;
    
}


void observer_get_xh(void)
{
    printf("%f,%f,%f,", obs_xh_f32[0],obs_xh_f32[1],obs_xh_f32[2]);
}

///*******************//////////////////// Onboard baby ///*******************////////////////////

void observer_set_y(void)
{
    obs_y_f32[0] = get_acc_angle();
    obs_y_f32[1] = get_gyroZ();
}

float observer_get_theta(void)
{
    theta = obs_xh_f32[1];
    return theta;
}

float observer_get_dtheta(void)
{
    dtheta = obs_xh_f32[0];// + obs_xh_f32[2];
    return dtheta;
}

void observer_set_u(float u)
{
    obs_u_f32[0] = u;
}

float observer_get_ptheta(void)
{
    // float A = 1/(0.2080+0.1120*cos(obs_xh_f32[1]));
    // float B = (0.1882+0.0560*cos(obs_xh_f32[1]))/(0.2080+0.1120*cos(obs_xh_f32[1]));
    // float ptheta = (obs_xh_f32[0]+get_motor_revs()*B)/A;
    // float A = 1/(0.2080+0.1120);
    // float B = (0.1882+0.0560)/(0.2080+0.1120);
    // float ptheta = B/A;
    //float A = 1/(0.2080+0.1120);
    //float B = (0.1882+0.0560)/(0.2080+0.1120);
    //float ptheta = (obs_xh_f32[0]+get_motor_revs()*B)/A;   // dthta/A + B+A*omega

    float A = 30.3771;
    float B = 0.2273;
    float ptheta = (obs_xh_f32[0]+get_motor_revs()*B)/A;   // dthta/A + B+A*omega
    //float ptheta = observer_get_dtheta()*0.04;
    //printf("ptheta = %f dtheta = %f\n", ptheta,obs_xh_f32[0]);
    return ptheta;
}