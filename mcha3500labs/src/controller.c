#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h" /* Include STM32 DSP matrix libraries */
#include "controller.h"
#include "observer.h"
#include "stepper.h"

#define CTRL_N_INPUT 1
#define CTRL_N_STATE 3 

/* Define control matrix values */
static float ctrl_mK_f32[CTRL_N_INPUT * CTRL_N_STATE] =
{
    /* negative K, 1x3 */
    32.9627243702405,	467.868808489669,	0.304073181583417, // half l
};

static float ctrl_x_f32[CTRL_N_STATE] =
{
    /* estimate of state, 3x1 */
    0.0,
    0.0,
    0.0,
};

static float ctrl_u_f32[CTRL_N_INPUT] =
{
    /* control action, 1x1 */
    0.0,
};

static float ctrl_Az_f32[CTRL_N_STATE] =
{
    /* State transition matrix */
    0,0,1,
};

static float ctrl_z_f32[CTRL_N_INPUT] =
{
    /* Integrator state */
    0.0102098022269040,
};

/* Define control matrix variables */
arm_matrix_instance_f32 ctrl_mK = {CTRL_N_INPUT, CTRL_N_STATE, (float32_t *)ctrl_mK_f32};
arm_matrix_instance_f32 ctrl_x = {CTRL_N_STATE, 1, (float32_t *)ctrl_x_f32};
arm_matrix_instance_f32 ctrl_u = {CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32};
arm_matrix_instance_f32 ctrl_Az = {1, CTRL_N_STATE, (float32_t *)ctrl_Az_f32};
arm_matrix_instance_f32 ctrl_z = {1, 1, (float32_t *)ctrl_z_f32};

/* Control functions */
void ctrl_init(void)
{
    arm_mat_init_f32(&ctrl_mK, CTRL_N_INPUT, CTRL_N_STATE, (float32_t *)ctrl_mK_f32);
    arm_mat_init_f32(&ctrl_x, CTRL_N_STATE, 1, (float32_t *)ctrl_x_f32);
    arm_mat_init_f32(&ctrl_u, CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32);
    arm_mat_init_f32(&ctrl_Az, 1, CTRL_N_STATE, (float32_t *)ctrl_Az_f32);
    arm_mat_init_f32(&ctrl_z, 1, 1, (float32_t *)ctrl_z_f32);
}

/* Update state vector elements */
void ctrl_set_x1_int(float x1)
{
    // Update state x1
    ctrl_x_f32[0] = x1;
}

void ctrl_set_x2_int(float x2)
{
    // Update state x2
    ctrl_x_f32[1] = x2;
}


/* Get the current control output */
float getControl(void)
{
    return ctrl_u_f32[0];
}

/* Update control output */
void ctrl_update(void)
{
    // Compute control action
    arm_mat_mult_f32(&ctrl_mK, &ctrl_x, &ctrl_u);

    // Update integrator state
    arm_mat_mult_f32(&ctrl_Az, &ctrl_x, &ctrl_z);

    // Copy updated value of integrator state into state vector
    ctrl_x_f32[2] = ctrl_z_f32[0];
}