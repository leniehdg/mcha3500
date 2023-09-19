#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h" /* Include STM32 DSP matrix libraries */
#include "qpas_sub_noblas.h"
#include "controller.h"

// Variables for QP solver
int numits, numadd, numdrop;

// Define actuator limits
#define u_min -1.0
#define u_max 1.0
#define delta_u_min -0.1
#define delta_u_max 0.1

/* Define control arrays */
// Hessian
static float32_t ctrl_H_f32[CTRL_N_HORIZON * CTRL_N_HORIZON] =
{
    0.00278506159446989, 0.000739686632904732, 0.000695421437418037, 0.000653269418994934, 0.000613079668343769, 0.000574706561323634, 0.000538009168974293, 0.000502850678720121, 0.000469097824232342, 0.000436620321444615,
    0.000739686632904732, 0.00270114033956792, 0.000660124206978452, 0.000619979305697565, 0.000581720093413039, 0.000545206052746511, 0.000510301323805813, 0.000476874139487119, 0.000444796270160728, 0.000413942475281437,
    0.000695421437418037, 0.000660124206978452, 0.00262571034717006, 0.000588600502718521, 0.000552146217127398, 0.000517371800246859, 0.000484145991796972, 0.000452341617879633, 0.000421835048542922, 0.000392505663085434,
    0.000653269418994934, 0.000619979305697565, 0.000588600502718521, 0.00255789063635690, 0.000524280281919088, 0.000491130324857741, 0.000459473852724703, 0.000429187841413093, 0.000400152832778554, 0.000372252411754383,
    0.000613079668343769, 0.000581720093413039, 0.000552146217127398, 0.000524280281919088, 0.00249688933225099, 0.000466412803335010, 0.000436219905396966, 0.000407351537176085, 0.000379691992187144, 0.000353128654883679,
    0.000574706561323634, 0.000545206052746511, 0.000517371800246859, 0.000491130324857741, 0.000466412803335010, 0.00244199468488632, 0.000414323301827425, 0.000386775274286689, 0.000360398442696156, 0.000335083591643094,
    0.000538009168974293, 0.000510301323805813, 0.000484145991796972, 0.000459473852724703, 0.000436219905396966, 0.000414323301827425, 0.00239256698478136, 0.000367405317827069, 0.000342221508061530, 0.000318069555291996,
    0.000502850678720121, 0.000476874139487119, 0.000452341617879633, 0.000429187841413093, 0.000407351537176085, 0.000386775274286689, 0.000367405317827069, 0.00234803128479971, 0.000325113791284899, 0.000302041891009477,
    0.000469097824232342, 0.000444796270160728, 0.000421835048542922, 0.000400152832778554, 0.000379691992187144, 0.000360398442696156, 0.000342221508061530, 0.000325113791284899, 0.00230787084681391, 0.000286958842744484,
    0.000436620321444615, 0.000413942475281437, 0.000392505663085434, 0.000372252411754383, 0.000353128654883679, 0.000335083591643094, 0.000318069555291996, 0.000302041891009477, 0.000286958842744484, 0.00227162123970961,

};

// f bar
static float32_t ctrl_fBar_f32[CTRL_N_HORIZON * CTRL_N_STATE] =
{
    -0.0514403441263408, -0.574232906371255, -0.153716835540947, -0.0890182423994215,
    -0.0473434854540904, -0.540856898023426, -0.144643750301427, -0.0837610684316850,
    -0.0434594156195519, -0.508990839585868, -0.135974639448588, -0.0787487545732824,
    -0.0397764898583356, -0.478536883071343, -0.127684476953097, -0.0739652334378638,
    -0.0362836026827773, -0.449400037955786, -0.119748896709951, -0.0693949572614822,
    -0.0329701559165030, -0.421487848323865, -0.112144118715652, -0.0650228426981379,
    -0.0298260280531692, -0.394710071457063, -0.104846874841926, -0.0608342162475701,
    -0.0268415448489395, -0.368978356653212, -0.0978343339608127, -0.0568147601024264,
    -0.0240074510612961, -0.344205923039134, -0.0910840261643521, -0.0529504581996810,
    -0.0213148832495485, -0.320307235105738, -0.0845737658094426, -0.0492275422580122,

};

// f
static float32_t ctrl_f_f32[CTRL_N_HORIZON] =
{
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
};

// State vector
static float32_t ctrl_xHat_f32[CTRL_N_STATE] =
{
    0.0,
    0.0,
    0.0,
    0.0,
};

static float ctrl_x_f32[CTRL_N_STATE] =
{
    /* estimate of state, 5x1 */
    0.0,
    0.0,
    0.0,
    0.0,
};

// Control
static float32_t ctrl_u_f32[CTRL_N_INPUT] =
{
    0.0,
};

// U star
static float32_t ctrl_Ustar_f32[CTRL_N_HORIZON * CTRL_N_INPUT] =
{
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
};

// Constraints
static float ctrl_A_f32[CTRL_N_INEQ_CONST * CTRL_N_HORIZON] =
{
    1, -1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
};

static float ctrl_b_f32[CTRL_N_INEQ_CONST] =
{
    0.1,
    0.100000000000000,
    0.100000000000000,
    0.100000000000000,
    0.100000000000000,
    0.100000000000000,
    0.100000000000000,
    0.100000000000000,
    0.100000000000000,
    0.100000000000000,
    0.1,
    0.100000000000000,
    0.100000000000000,
    0.100000000000000,
    0.100000000000000,
    0.100000000000000,
    0.100000000000000,
    0.100000000000000,
    0.100000000000000,
    0.100000000000000,
};

static float ctrl_xl_f32[CTRL_N_LB_CONST] =
{
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
};

static float ctrl_xu_f32[CTRL_N_UB_CONST] =
{
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
};

static float ctrl_lm_f32[CTRL_N_EQ_CONST + CTRL_N_INEQ_CONST + CTRL_N_LB_CONST + CTRL_N_UB_CONST] =
{
    // ...
};


/* Define control matrix variables */
arm_matrix_instance_f32 ctrl_fBar = {CTRL_N_HORIZON, CTRL_N_STATE, (float32_t *)ctrl_fBar_f32};
arm_matrix_instance_f32 ctrl_f = {CTRL_N_HORIZON, 1, (float32_t *)ctrl_f_f32};
arm_matrix_instance_f32 ctrl_xHat = {CTRL_N_STATE, 1, (float32_t *)ctrl_xHat_f32};
arm_matrix_instance_f32 ctrl_u = {CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32};
arm_matrix_instance_f32 ctrl_x = {CTRL_N_STATE, 1, (float32_t *)ctrl_x_f32};


/* Control functions */
void ctrl_init(void)
{
    arm_mat_init_f32(&ctrl_fBar, CTRL_N_HORIZON, CTRL_N_STATE, (float32_t *)ctrl_fBar_f32);
    arm_mat_init_f32(&ctrl_f, CTRL_N_HORIZON, 1, (float32_t *)ctrl_f_f32);
    arm_mat_init_f32(&ctrl_xHat, CTRL_N_STATE, 1, (float32_t *)ctrl_xHat_f32);
    arm_mat_init_f32(&ctrl_u, CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32);
    arm_mat_init_f32(&ctrl_x, CTRL_N_STATE, 1, (float32_t *)ctrl_x_f32);
}


/* Update state vector elements */
void ctrl_set_x1(float x1)
{
    // Update state x1
    ctrl_x_f32[0] = x1;
}

void ctrl_set_x2(float x2)
{
    // Update state x2
    ctrl_x_f32[1] = x2;
}

void ctrl_set_x3(float x3)
{
    // Update state x3
    ctrl_x_f32[2] = x3;
}

void ctrl_set_x4(float x4)
{
    // Update state x4
    ctrl_x_f32[3] = x4;
}

/* Get the current control output USED FOR CMD_PARSER */
float getControl(void)
{
    return ctrl_u_f32[0];
}


void ctrl_update(void)
{
    // TODO: Compute f vector
    arm_mat_mult_f32(&ctrl_fBar, &ctrl_x, &ctrl_f);

    // TODO: Update b vector
    ctrl_b_f32[0] = ctrl_u_f32[0] + delta_u_max;
    ctrl_b_f32[CTRL_N_HORIZON] = - ctrl_u_f32[0] - delta_u_min;

    // TODO: Solve for optimal inputs over control horizon
    qpas_sub_noblas(CTRL_N_HORIZON, CTRL_N_EQ_CONST, CTRL_N_INEQ_CONST, CTRL_N_LB_CONST, CTRL_N_UB_CONST, ctrl_H_f32, ctrl_f_f32, ctrl_A_f32, ctrl_b_f32, ctrl_xl_f32, ctrl_xu_f32, ctrl_Ustar_f32, ctrl_lm_f32, 0, &numits, &numadd, &numdrop);

    // TODO: Extract first control term
    ctrl_u_f32[0] = ctrl_Ustar_f32[0];

    /* Print functions for debugging. Uncomment to use */
    // printmatrix (CTRL_N_HORIZON, CTRL_N_HORIZON, ctrl_H_f32, CTRL_N_HORIZON, "H");
    // printvector (CTRL_N_HORIZON, ctrl_f_f32, "f");
}