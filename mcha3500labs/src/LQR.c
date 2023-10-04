// #include <stdint.h>
// #include <stdlib.h>
// #include <math.h>
// #include "stm32f4xx_hal.h"
// #include "cmsis_os2.h"
// #include "uart.h"
// #include "LQR.h"

// #define CTRL_LQR_N_INPUT 1
// #define CTRL_LQR_N_STATE 5
// #define CTRL_LQR_N_TEMP 4

// /* Define control matrix values */
// static float ctrl_LQR_xStar_f32[CTRL_LQR_N_STATE] = 
// {
// 1,	0,	0,	0, 0,
// };


// static float ctrl_LQR_mK_f32[CTRL_LQR_N_INPUT*CTRL_LQR_N_STATE] = 
// {
// /* negative K, 1x5 */
// 3.07262805257052,	18.3391931397558,	4.39980123905778,	6.19934220125095,	0.644623127056843,
// };


// static float ctrl_LQR_x_f32[CTRL_LQR_N_STATE] =
// {
// /* estimate of state, 5x1 */
//  0.0,
//  0.0175,
//  0.0,
//  0.0,
//  0.0,
//  };

//  static float ctrl_LQR_u_f32[CTRL_LQR_N_INPUT] =
//  {
//  /* control action, 1x1 */
//  0.0,
//  };
//  static float ctrl_LQR_Az_f32[CTRL_LQR_N_STATE] =
//  {
//  /* State transition matrix */
//  0.00500000000000000,	0,	0,	0,	1,
//  };
//  static float ctrl_LQR_z_f32[CTRL_LQR_N_INPUT] =
//  {
//  /* Integrator state */
//  0.0,
//  };

//  /* Define control matrix variables */
//  // rows, columns, data array
//  arm_matrix_instance_f32 ctrl_LQR_mK = {CTRL_N_LQR_INPUT, CTRL_N_LQR_STATE, (float32_t *)ctrl_mK_LQR_f32};


//  arm_matrix_instance_f32 ctrl_LQR_x = {CTRL_N_LQR_STATE, 1, (float32_t *)ctrl_x_LQR_f32};
//  arm_matrix_instance_f32 ctrl_LQR_u = {CTRL_N_LQR_INPUT, 1, (float32_t *)ctrl_u_LQR_f32};
//  arm_matrix_instance_f32 ctrl_LQR_Az = {1, CTRL_N_LQR_STATE, (float32_t *)ctrl_Az_LQR_f32};
//  arm_matrix_instance_f32 ctrl_LQR_z = {1, 1, (float32_t *)ctrl_z_LQR_f32};

//  /* Control functions */
//  void ctrl_LQR_init(void)
//  {
//  arm_mat_init_f32(&ctrl_LQR_mK, CTRL_N_LQR_INPUT, CTRL_N_LQR_STATE, (float32_t *)ctrl_mK_LQR_f32);


//  arm_mat_init_f32(&ctrl_LQR_x, CTRL_N_LQR_STATE, 1, (float32_t *)ctrl_x_LQR_f32);
//  arm_mat_init_f32(&ctrl_LQR_u, CTRL_N_LQR_INPUT, 1, (float32_t *)ctrl_u_LQR_f32);
//  arm_mat_init_f32(&ctrl_LQR_Az, 1, CTRL_N_LQR_STATE, (float32_t *)ctrl_Az_LQR_f32);
//  arm_mat_init_f32(&ctrl_LQR_z, 1, 1, (float32_t *)ctrl_z_LQR_f32);
//  }

// /* Update control output */
// void ctrl_LQR_update(void)
// {
// // TODO: Compute control action
// arm_mat_mult_f32(&ctrl_LQR_mK, &ctrl_LQR_x, &ctrl_LQR_u);

// // TODO: Update integrator state
// arm_mat_mult_f32(&ctrl_LQR_Az, &ctrl_LQR_x, &ctrl_LQR_z);
// // Copy updated value of integrator state into state vector
//  ctrl_x_f32[4] = ctrl_z_f32[0];
//  }