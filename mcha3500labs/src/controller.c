#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "arm_math.h" /* Include STM32 DSP matrix libraries */
#include "controller.h"
#include "observer.h"
#include "stepper.h"

#define CTRL_N_INPUT 1
#define CTRL_N_STATE 2
#define CTRL_N_INT_STATE 3

float slew = 1;

/* Define control matrix values */

static float ctrl_mK_f32[CTRL_N_INPUT*CTRL_N_INT_STATE] = 
{
    /* negative K, 1x3 */
    // FROM RUN_BALANCINGROBOTFLOW.M -> CONTROLLERBALANCINGROBOT OBJ -> K VALUE THEN TAKE THE NEGATIVE!!!
    49.2341729908953,	333.301261681590,	0.305179489453004, 
};

static float ctrl_x_int_f32[CTRL_N_INT_STATE] =
{
    /* estimate of state, 5x1 */
    0.0,
    0.0,
    0.0,
};

static float ctrl_u_f32[CTRL_N_INPUT] =
{
    /* control action, 1x1 */
    0.0,
};

static float ctrl_u_prev_f32[CTRL_N_INPUT] =
 {
    /* control action, 1x1 */
    0.0,
 };
 
static float ctrl_Az_f32[CTRL_N_INT_STATE] =
{
    /* State transition matrix */
    0,	0,	1,
};

static float ctrl_z_f32[CTRL_N_INPUT] =
{
    /* Integrator state */
    0.0,
};

static float ctrl_Azmx_f32[CTRL_N_INPUT] =
{
    /* Integrator state */
    0.0,
};

static float ctrl_Bz_f32[CTRL_N_INPUT] =
{
    /* Integrator state */
    0.0050,
};

static float ctrl_Bzmu_f32[CTRL_N_INPUT] =
{
    /* Integrator state */
    0,
};

/* Define control matrix variables */
// rows, columns, data array
arm_matrix_instance_f32 ctrl_mK = {CTRL_N_INPUT, CTRL_N_INT_STATE, (float32_t *)ctrl_mK_f32};
arm_matrix_instance_f32 ctrl_x_int = {CTRL_N_INT_STATE, 1, (float32_t *)ctrl_x_int_f32};
arm_matrix_instance_f32 ctrl_u = {CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32};
arm_matrix_instance_f32 ctrl_Az = {1, CTRL_N_INT_STATE, (float32_t *)ctrl_Az_f32};
arm_matrix_instance_f32 ctrl_z = {1, 1, (float32_t *)ctrl_z_f32};
arm_matrix_instance_f32 ctrl_Azmx = {1, 1, (float32_t *)ctrl_Azmx_f32};
arm_matrix_instance_f32 ctrl_Bz = {1, 1, (float32_t *)ctrl_Bz_f32};
arm_matrix_instance_f32 ctrl_Bzmu = {1, 1, (float32_t *)ctrl_Bzmu_f32};
arm_matrix_instance_f32 ctrl_u_prev = {CTRL_N_INPUT, 1, (float32_t *)ctrl_u_prev_f32};


/* Control functions */
void ctrl_init(void)
{
arm_mat_init_f32(&ctrl_mK, CTRL_N_INPUT, CTRL_N_INT_STATE, (float32_t *)ctrl_mK_f32);
arm_mat_init_f32(&ctrl_x_int, CTRL_N_INT_STATE, 1, (float32_t *)ctrl_x_int_f32);
arm_mat_init_f32(&ctrl_u, CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32);
arm_mat_init_f32(&ctrl_Az, 1, CTRL_N_INT_STATE, (float32_t *)ctrl_Az_f32);
arm_mat_init_f32(&ctrl_z, 1, 1, (float32_t *)ctrl_z_f32);
arm_mat_init_f32(&ctrl_Azmx, 1, 1, (float32_t *)ctrl_Azmx_f32);
arm_mat_init_f32(&ctrl_Bz, 1, 1, (float32_t *)ctrl_Bz_f32);
arm_mat_init_f32(&ctrl_Bzmu, 1, 1, (float32_t *)ctrl_Bzmu_f32);
arm_mat_init_f32(&ctrl_u_prev, CTRL_N_INPUT, 1, (float32_t *)ctrl_u_prev_f32);
}


/* Update state vector elements */
void ctrl_set_x1_int(float x1)
{
    // Update state x1
    ctrl_x_int_f32[0] = x1;
    // printf("theta (rad): %f\n", ctrl_x_int_f32[0]);

}

void ctrl_set_x2_int(float x2)
{
    // Update state x2
    ctrl_x_int_f32[1] = x2;
    // printf("dtheta: %f\n \n", ctrl_x_int_f32[1]);

}


/* Update control output */
void ctrl_update(void)
{
    /*      Compute control action      */   
    arm_mat_mult_f32(&ctrl_mK, &ctrl_x_int, &ctrl_u);   // u = -K*x

    /*      Limit the control action using a slew       */   

    if(fabs(ctrl_u_f32[0]) > fabs(ctrl_u_prev_f32[0])+slew)
    {
        if(ctrl_u_prev_f32[0]<ctrl_u_f32[0])
        {
            ctrl_u_f32[0] = ctrl_u_prev_f32[0] + slew;
        }
        if(ctrl_u_prev_f32[0]>ctrl_u_f32[0])
        {
            ctrl_u_f32[0] = ctrl_u_prev_f32[0] - slew;
        }
    }
    else if(fabs(ctrl_u_f32[0]) < fabs(ctrl_u_prev_f32[0])-slew)
    {
        if(ctrl_u_prev_f32[0]<ctrl_u_f32[0])
        {
            ctrl_u_f32[0] = ctrl_u_prev_f32[0] + slew;
        }
        if(ctrl_u_prev_f32[0]>ctrl_u_f32[0])
        {
            ctrl_u_f32[0] = ctrl_u_prev_f32[0] - slew;
        }
    }

    ctrl_u_prev_f32[0] = ctrl_u_f32[0];
    // printf("Control action with slew (u): %f\n", ctrl_u_f32[0]);


    /*      Update integrator state     */

    arm_mat_mult_f32(&ctrl_Az, &ctrl_x_int, &ctrl_Azmx);    // Az*x
    arm_mat_mult_f32(&ctrl_Bz, &ctrl_u, &ctrl_Bzmu);        // Bz*u
    arm_mat_add_f32(&ctrl_Azmx, &ctrl_Bzmu, &ctrl_z);       // z = Az*x + Bz*u
    
    // Copy updated value of integrator state into state vector
    ctrl_x_int_f32[2] = ctrl_z_f32[0];
    // printf("Integrator state: %f\n\n", ctrl_x_int_f32[2]);

}


/*      Get the current control output      */

float getControl(void)
{
    return ctrl_u_f32[0];
}


/*      Get current states      */

float ctrl_get_dtheta(void)
{
    return ctrl_x_int_f32[0];
}

float ctrl_get_dPtheta(void)
{
    return ctrl_x_int_f32[1];
}


