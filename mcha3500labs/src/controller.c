#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h" /* Include STM32 DSP matrix libraries */
#include "controller.h"
#include"qpas_sub_noblas.h"

#define CTRL_N_INPUT 1
#define CTRL_Nx_SIZE 2 
#define CTRL_N_STATE 3 

/* Define control matrix values */
static float ctrl_mK_f32[CTRL_N_INPUT * CTRL_N_STATE] =
{
    /* negative K, 1x3 */
    // 43.7414767910140,	480.459793467096,	0.949857748811064,  // half l_opt
    // 47.1963556855334,	319.391283628879,	3.05405420741840,
    116.884185690572,	771.039591492346,	9.38549057321377,
};

static float ctrl_x_f32[CTRL_N_STATE] =
{
    /* estimate of state, 3x1 */
    3.141592653/180,
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
    0.005,  0,  1,
};

static float ctrl_z_f32[CTRL_N_INPUT] =
{
    /* Integrator state */
    // -0.361754064429894,  // half l_opt
    // -0.251670465943562,
    0,
};

static float ctrl_Nx_f32[CTRL_N_STATE] =
{
    /* Integrator state */
    0.000,
    0.0062,     // 1.000,
    0.000,
};

static float ctrl_Nu_f32[CTRL_N_INPUT] =
{
    
    // 246.125322879505,    // half l_opt
    // 161.990707694636,
    1,

};

static float ctrl_xstar_f32[CTRL_N_STATE] =
{
    /* Integrator state */
    0.000,
    0.000,
    0.000,
};
static float ctrl_ustar_f32[CTRL_N_INPUT] =
{
    0.000,
};
static float ctrl_yref_f32[CTRL_N_INPUT] =
{
    /* Integrator state */
    1.000,
};

static float ctrl_x_sub_xstar_f32[CTRL_N_STATE] =
{
    /* Integrator state */
    0.000,
    0.000,
    0.000,
};

static float ctrl_Kx_sub_xstar_f32[CTRL_N_INPUT] =
{
    /* Integrator state */
    0.000,
};



/* Define control matrix variables */
arm_matrix_instance_f32 ctrl_Kx_sub_xstar = {CTRL_N_INPUT, 1, (float32_t *)ctrl_Kx_sub_xstar_f32};
arm_matrix_instance_f32 ctrl_x_sub_xstar = {CTRL_N_STATE, 1, (float32_t *)ctrl_x_sub_xstar_f32};
arm_matrix_instance_f32 ctrl_xstar = {CTRL_N_STATE, 1, (float32_t *)ctrl_xstar_f32};
arm_matrix_instance_f32 ctrl_ustar = {CTRL_N_INPUT, 1, (float32_t *)ctrl_ustar_f32};
arm_matrix_instance_f32 ctrl_yref = {CTRL_N_INPUT, 1, (float32_t *)ctrl_yref_f32};
arm_matrix_instance_f32 ctrl_Nx = {CTRL_Nx_SIZE, 1, (float32_t *)ctrl_Nx_f32};
arm_matrix_instance_f32 ctrl_Nu = {CTRL_N_INPUT, 1, (float32_t *)ctrl_Nu_f32};
arm_matrix_instance_f32 ctrl_mK = {CTRL_N_INPUT, CTRL_N_STATE, (float32_t *)ctrl_mK_f32};
arm_matrix_instance_f32 ctrl_x = {CTRL_N_STATE, 1, (float32_t *)ctrl_x_f32};
arm_matrix_instance_f32 ctrl_u = {CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32};
arm_matrix_instance_f32 ctrl_Az = {1, CTRL_N_STATE, (float32_t *)ctrl_Az_f32};
arm_matrix_instance_f32 ctrl_z = {1, 1, (float32_t *)ctrl_z_f32};

/* Control functions */
void ctrl_init(void)
{
    arm_mat_init_f32(&ctrl_Kx_sub_xstar, CTRL_N_INPUT, 1, (float32_t *)ctrl_Kx_sub_xstar_f32);
    arm_mat_init_f32(&ctrl_x_sub_xstar, CTRL_N_STATE, 1, (float32_t *)ctrl_x_sub_xstar_f32);
    arm_mat_init_f32(&ctrl_xstar, CTRL_N_STATE, 1, (float32_t *)ctrl_xstar_f32);
    arm_mat_init_f32(&ctrl_ustar, CTRL_N_INPUT, 1, (float32_t *)ctrl_ustar_f32);
    arm_mat_init_f32(&ctrl_yref, CTRL_N_INPUT, 1, (float32_t *)ctrl_yref_f32);
    arm_mat_init_f32(&ctrl_Nx, CTRL_N_STATE, 1, (float32_t *)ctrl_Nx_f32);
    arm_mat_init_f32(&ctrl_Nu, CTRL_N_INPUT, 1, (float32_t *)ctrl_Nu_f32);
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
    arm_mat_mult_f32(&ctrl_Nx, &ctrl_yref, &ctrl_xstar);
    //printvector(3,ctrl_xstar_f32,"xstar");
    
    arm_mat_mult_f32(&ctrl_Nu, &ctrl_yref, &ctrl_ustar);
    //printvector(1,ctrl_ustar_f32,"ustar");

    arm_mat_sub_f32(&ctrl_x, &ctrl_xstar, &ctrl_x_sub_xstar);
    //printvector(3,ctrl_x_f32,"x");
    //printvector(3,ctrl_xstar_f32,"xstar");
    //printvector(3,ctrl_x_sub_xstar_f32,"xsub_xstar");

    // Compute control action
    arm_mat_mult_f32(&ctrl_mK, &ctrl_x_sub_xstar, &ctrl_Kx_sub_xstar);
    //printvector(3,ctrl_mK_f32,"K");
    
    //printvector(1,ctrl_Kx_sub_xstar_f32,"Kxsub_xstar");
    arm_mat_add_f32(&ctrl_ustar, &ctrl_Kx_sub_xstar, &ctrl_u);
    //printvector(1,ctrl_u_f32,"u");

    // Update integrator state
    arm_mat_mult_f32(&ctrl_Az, &ctrl_x_sub_xstar, &ctrl_z);
    //printvector(1 ,ctrl_z_f32,"z");
    // Copy updated value of integrator state into state vector
    ctrl_x_f32[2] = ctrl_z_f32[0];
}

/*
case 'Reference_integrator'
                    % TODO: Compute feed-forward terms u_star and x_star
                    x_star = obj.N_x*obj.y_ref;
                    u_star = obj.N_u*obj.y_ref;
                    % TODO: Compute control value
                    u = u_star - obj.K*[(x - x_star);obj.z];
                    % Update integrator state for next iteration
                    obj.z = obj.z + (obj.T*obj.C_r*(x-x_star)) + (obj.T*obj.D_r*(-obj.K*[(x - x_star);obj.z]));
                    obj.z = obj.z + (Az*(x-x_star)) + (Bz*(-obj.K*[(x - x_star);obj.z])); Notes Bz is 0 so that term cancels
*/