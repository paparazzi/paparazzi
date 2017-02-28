/*
 * File: UKF_Wind_Estimator.h
 *
 * Code generated for Simulink model 'UKF_Wind_Estimator'.
 *
 * Model version                  : 1.120
 * Simulink Coder version         : 8.10 (R2016a) 10-Feb-2016
 * C/C++ source code generated on : Wed Nov  2 23:49:42 2016
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_UKF_Wind_Estimator_h_
#define RTW_HEADER_UKF_Wind_Estimator_h_
#include <math.h>
#include <string.h>
#ifndef UKF_Wind_Estimator_COMMON_INCLUDES_
# define UKF_Wind_Estimator_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* UKF_Wind_Estimator_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */

/* Block signals and states (auto storage) for system '<Root>' */
typedef struct {
  real32_T Delay1_DSTATE[7];           /* '<Root>/ Delay1' */
  real32_T Delay_DSTATE[49];           /* '<Root>/ Delay' */
  boolean_T x_not_empty;               /* '<Root>/initialization' */
  boolean_T P_not_empty;               /* '<Root>/initialization' */
} DW;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real32_T rates[3];                   /* '<Root>/rates' */
  real32_T accel[3];                   /* '<Root>/accel' */
  real32_T q[4];                       /* '<Root>/q' */
  real32_T vk[3];                      /* '<Root>/vk' */
  real32_T va;                         /* '<Root>/va ' */
  real32_T aoa;                        /* '<Root>/aoa' */
  real32_T sideslip;                   /* '<Root>/sideslip' */
} ExtU;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real32_T xout[7];                    /* '<Root>/xout' */
  real32_T Pout[49];                   /* '<Root>/Pout' */
} ExtY;

/* Type definition for custom storage class: Struct */
typedef struct ukf_init_tag {
  real32_T x0[7];
  real32_T P0[49];
  real32_T ki;
  real32_T alpha;
  real32_T beta;
} ukf_init_type;

typedef struct ukf_params_tag {
  real32_T Q[49];
  real32_T R[36];
  real32_T dt;
} ukf_params_type;

/* Block signals and states (auto storage) */
extern DW ukf_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU ukf_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY ukf_Y;

/* Model entry point functions */
extern void UKF_Wind_Estimator_initialize(void);
extern void UKF_Wind_Estimator_step(void);

/* Exported data declaration */

/* Declaration for custom storage class: Struct */
extern ukf_init_type ukf_init;
extern ukf_params_type ukf_params;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'UKF_Wind_Estimator'
 * '<S1>'   : 'UKF_Wind_Estimator/UKF_correction'
 * '<S2>'   : 'UKF_Wind_Estimator/UKF_prediction'
 * '<S3>'   : 'UKF_Wind_Estimator/initialization'
 * '<S4>'   : 'UKF_Wind_Estimator/main'
 * '<S5>'   : 'UKF_Wind_Estimator/sigmas'
 */
#endif                                 /* RTW_HEADER_UKF_Wind_Estimator_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
