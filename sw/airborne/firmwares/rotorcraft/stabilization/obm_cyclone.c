#include "firmwares/rotorcraft/stabilization/stabilization_andi.h"
#include<math.h>

#if ANDI_NUM_ACT != 4
  #error Cyclone expects 4 actuators
#endif

#if ANDI_OUTPUTS != 4
  #error The cyclone model provides 4 outputs
#endif

union CycloneCoefficients {
  struct {
    // X-axis force coefficients (f_ff_x)
    float fx_motor_squared;          // Motor thrust squared effect
    float fx_speed_forward;          // Forward speed effect

    // Y-axis force coefficients (f_ff_y)
    float fy_speed_lateral;          // Lateral speed effect

    // Z-axis force coefficients (f_ff_z)
    float fz_motor_squared;          // Motor thrust squared effect
    float fz_speed_forward;          // Forward speed effect
    float fz_speed_vertical;         // Vertical speed effect
    float fz_elevon_speed;           // Elevon-speed coupling
    float fz_elevon_motor;           // Elevon-motor coupling

    // X-axis moment coefficients (m_ff_x)
    float mx_elevon_motor_diff;      // (ele_l * motor_l^2 - ele_r * motor_r^2)
    float mx_elevon_speed_diff;      // (ele_l - ele_r) * speed * v_ff(1)
    float mx_angular_drag;           // w_ff(1)^2
    float mx_angular_coupling;       // w_ff(2) * w_ff(3)

    // Y-axis moment coefficients (m_ff_y)
    float my_speed_forward;          // speed * v_ff(1)
    float my_speed_vertical;         // speed * v_ff(3)
    float my_motor_sum;              // motor_l^2 + motor_r^2
    float my_elevon_motor_sum;       // ele_l * motor_l^2 + ele_r * motor_r^2
    float my_elevon_speed_sum;       // (ele_l + ele_r) * speed * v_ff(1)
    float my_angular_sum;            // w_ff(1) + w_ff(3)

    // Z-axis moment coefficients (m_ff_z)
    float mz_speed_lateral;          // speed * v_ff(2)
    float mz_motor_diff;             // motor_l^2 - motor_r^2
    float mz_speed_roll;             // speed * w_ff(1)
    float mz_angular_coupling;       // w_ff(1) * w_ff(2)

    // Fixme: Move to x-axis moment coefficients
  };
  float data[22];
};

// Model coefficinets
union CycloneCoefficients obm_coefficients = {
  .fx_motor_squared     = 0.00000735f,
  .fx_speed_forward     = -0.03f,

  .fy_speed_lateral     = -0.008f,

  .fz_motor_squared     = 0.0f,
  .fz_speed_forward     = 0.0f,
  .fz_speed_vertical    = -0.144f,
  .fz_elevon_speed      = 0.0f,
  .fz_elevon_motor      = 0.0f,

  .mx_elevon_motor_diff = 1.9e-05f,
  .mx_elevon_speed_diff = 0.344f,
  .mx_angular_drag      = -0.4940217f,
  .mx_angular_coupling  = -2.18f,

  .my_speed_forward     = 0.0f,
  .my_speed_vertical    = -0.0888f,
  .my_motor_sum         = 0.0f,
  .my_elevon_motor_sum  = -0.0000424f,
  .my_elevon_speed_sum  = 0.2525f,
  .my_angular_sum       = 1.262f,

  .mz_speed_lateral     = -0.00371f,
  .mz_motor_diff        = 0.000039f,
  .mz_speed_roll        = -0.0129f,
  .mz_angular_coupling  = -0.4827f,
};

union CeMatrix {
  struct {
    float ce_11;
    float ce_12;
    float ce_13;
    float ce_14;
    float ce_21;
    float ce_22;
    float ce_23;
    float ce_24;
    float ce_31;
    float ce_32;
    float ce_33;
    float ce_34;
    float ce_41;
    float ce_42;
    float ce_43;
    float ce_44;

  };
  float data[16];
};

// Model coefficinets
union CeMatrix ce_mat_tmp = {
  .ce_11 = 0.0f,        .ce_12 = 0.0f,        .ce_13 = 3.9e-5f,     .ce_14 = -3.9e-5f, // Roll
  .ce_21 = -29.917439f, .ce_22 = -29.917439f, .ce_23 = 0.0f,        .ce_24 = 0.0f,     // Pitch
  .ce_31 = -19.968481f, .ce_32 = 19.968481f,  .ce_33 = 0.0f,        .ce_34 = 0.0f,     // Yaw
  .ce_41 = 0.0f,        .ce_42 = 0.0f,        .ce_43 = 7e-6f,       .ce_44 = 7e-6f,    // Thrust
};


/* Function Definitions */
static void cyclone_obm_forces(const float body_vel[3], const float u[4],
                               const float coeff[22], float F_obm_forces[3])
{
  float t2;
  float t7;
  t2 = u[2] + u[3];
  t7 = sqrtf((body_vel[0] * body_vel[0] + body_vel[1] * body_vel[1]) + body_vel[2] * body_vel[2]);
  F_obm_forces[0] = coeff[3] * t2
                    + coeff[7] * (u[0] * u[2] + u[1] * u[3])
                    + coeff[5] * t7 * body_vel[0]
                    - coeff[4] * t7 * body_vel[2]
                    - coeff[6] * t7 * body_vel[2] * (u[0] + u[1]);
  F_obm_forces[1] = coeff[2] * t7 * body_vel[1];
  F_obm_forces[2] = -coeff[0] * t2 + coeff[1] * t7 * body_vel[2];
}

static void cyclone_obm_moments(const float rates[3],
                                const float body_vel[3], const float u[4],
                                const float coeff[22], float F_obm_moments[3])
{
  float t2;
  float t3;
  float t8;
  t2 = u[0] * u[2];
  t3 = u[1] * u[3];
  t8 = sqrtf((body_vel[0] * body_vel[0] + body_vel[1] * body_vel[1]) + body_vel[2] * body_vel[2]);
  F_obm_moments[0] = coeff[19] * (u[2] - u[3])
                     + coeff[18] * t8 * body_vel[1]
                     - coeff[20] * t8 * rates[2]
                     - rates[1] * coeff[21] * rates[2];
  F_obm_moments[1] = coeff[15] * (t2 + t3)
                     + coeff[14] * (u[2] + u[3])
                     + coeff[13] * t8 * body_vel[0]
                     - coeff[12] * t8 * body_vel[2]
                     - rates[0] * coeff[17] * rates[2]
                     - coeff[16] * t8 * body_vel[2] * (u[0] + u[1]);
  F_obm_moments[2] = -coeff[8] * (t2 - t3)
                     + coeff[10] * fabsf(rates[2]) * rates[2]
                     - rates[0] * coeff[11] * rates[1]
                     + coeff[9] * t8 * body_vel[2] * (u[0] - u[1]);
}


static void cyclone_f_stb_u(const float body_vel[3],
                            const float u[4], const float coeff[22], float F_stb_u[16])
{
  float fv[16];
  float t6;
  float t7;
  int i;
  int i1;
  int i2;
  t6 = sqrtf((body_vel[0] * body_vel[0] + body_vel[1] * body_vel[1]) + body_vel[2] * body_vel[2]);
  t7 = coeff[9] * t6 * body_vel[2];
  t6 = coeff[16] * t6 * body_vel[2];
  fv[0] = 0.0F;
  fv[1] = -t6 + u[2] * coeff[15];
  fv[2] = t7 - u[2] * coeff[8];
  fv[3] = 0.0F;
  fv[4] = 0.0F;
  fv[5] = -t6 + u[3] * coeff[15];
  fv[6] = -t7 + u[3] * coeff[8];
  fv[7] = 0.0F;
  fv[8] = coeff[19];
  fv[9] = coeff[14] + u[0] * coeff[15];
  fv[10] = -coeff[8] * u[0];
  fv[11] = coeff[0];
  fv[12] = -coeff[19];
  fv[13] = coeff[14] + u[1] * coeff[15];
  fv[14] = u[1] * coeff[8];
  fv[15] = coeff[0];
  i = 0;
  i1 = 0;
  for (i2 = 0; i2 < 16; i2++) {
    F_stb_u[i1 + (i << 2)] = fv[i2];
    i++;
    if (i > 3) {
      i = 0;
      i1++;
    }
  }
}

/* Function Definitions */
void cyclone_f_stb_x(const float rates[3], const float ang_accel[3],
                     const float vel_body[3], const float accel_body[3], const float u[4],
                     const float coeff[22], float F_stb_x[4])
{
  float F_stb_x_tmp;
  float b_F_stb_x_tmp;
  float c_F_stb_x_tmp;
  float d_F_stb_x_tmp;
  float et1_tmp;
  float t10;
  float t12;
  float t15;
  float t8;
  float t9;
  t8 = vel_body[0] * vel_body[0];
  t9 = vel_body[1] * vel_body[1];
  t10 = vel_body[2] * vel_body[2];
  t12 = u[0] - u[1];
  t15 = sqrtf((t8 + t9) + t10);
  et1_tmp = fmaxf(1.0E-8F, t15);
  F_stb_x[0] = -(
                 -accel_body[1] * coeff[18] * t9
                 - accel_body[0] * coeff[18] * vel_body[0] * vel_body[1]
                 - accel_body[2] * coeff[18] * vel_body[1] * vel_body[2]
                 + accel_body[0] * coeff[20] * vel_body[0] * rates[2]
                 + accel_body[1] * coeff[20] * vel_body[1] * rates[2]
                 + accel_body[2] * coeff[20] * vel_body[2] * rates[2]
                 - accel_body[1] * coeff[18] * t15 * et1_tmp
                 + coeff[20] * t15 * ang_accel[2] * et1_tmp
                 + rates[1] * coeff[21] * ang_accel[2] * et1_tmp
                 + rates[2] * coeff[21] * ang_accel[1] * et1_tmp
               ) / et1_tmp;

  t9 = accel_body[2] * coeff[12];
  F_stb_x_tmp = accel_body[2] * coeff[16];
  b_F_stb_x_tmp = accel_body[0] * coeff[16];
  c_F_stb_x_tmp = accel_body[1] * coeff[16];
  d_F_stb_x_tmp = F_stb_x_tmp * u[0];
  F_stb_x_tmp *= u[1];
  F_stb_x[1] = -(
                 -accel_body[0] * coeff[13] * t8 + t9 * t10
                 + accel_body[0] * coeff[12] * vel_body[0] * vel_body[2]
                 - accel_body[1] * coeff[13] * vel_body[0] * vel_body[1]
                 + accel_body[1] * coeff[12] * vel_body[1] * vel_body[2]
                 - accel_body[1] * coeff[12] * vel_body[1] * vel_body[2]
                 - accel_body[2] * coeff[13] * vel_body[0] * vel_body[2]
                 - accel_body[0] * coeff[13] * t15 * et1_tmp
                 + t9 * t15 * et1_tmp
                 + rates[0] * coeff[17] * ang_accel[2] * et1_tmp
                 + rates[2] * coeff[17] * ang_accel[0] * et1_tmp
                 + d_F_stb_x_tmp * t10
                 + F_stb_x_tmp * t10
                 + b_F_stb_x_tmp * u[0] * vel_body[0] * vel_body[2]
                 + c_F_stb_x_tmp * u[0] * vel_body[1] * vel_body[2]
                 + b_F_stb_x_tmp * u[1] * vel_body[0] * vel_body[2]
                 + c_F_stb_x_tmp * u[1] * vel_body[1] * vel_body[2]
                 + d_F_stb_x_tmp * t15 * et1_tmp
                 + F_stb_x_tmp * t15 * et1_tmp
               ) / et1_tmp;
  F_stb_x[2] = accel_body[2] * (coeff[9] * t12 * t15 + coeff[9] * t10 * t12 / et1_tmp)
               - rates[0] * coeff[11] * ang_accel[1]
               - rates[1] * coeff[11] * ang_accel[0]
               + 2.0F * coeff[10] * fabsf(rates[2]) * ang_accel[2]
               + accel_body[0] * coeff[9] * t12 * vel_body[0] * vel_body[2] / et1_tmp
               + accel_body[1] * coeff[9] * t12 * vel_body[1] * vel_body[2] / et1_tmp;
  F_stb_x[3] = 0.0F;
}



/* End of code generation (cyclone_f_stb_x.c) */

struct FloatVect3 evaluate_obm_forces(const struct FloatRates *rates, const struct FloatVect3 *vel_body,
                                      const float actuator_state[ANDI_NUM_ACT])
{
  float vel_body_array[3];
  (void)rates;

  vel_body_array[0] = vel_body->x;
  vel_body_array[1] = vel_body->y;
  vel_body_array[2] = vel_body->z;

  float forces_array[3];
  cyclone_obm_forces(vel_body_array, actuator_state, obm_coefficients.data, forces_array);

  struct FloatVect3 forces;
  forces.x = forces_array[0];
  forces.y = forces_array[1];
  forces.z = forces_array[2];

  return forces;
}

struct FloatVect3 evaluate_obm_moments(const struct FloatRates *rates, const struct FloatVect3 *vel_body,
                                       const float actuator_state[ANDI_NUM_ACT])
{
  float vel_body_array[3];
  float rates_array[3];

  vel_body_array[0] = vel_body->x;
  vel_body_array[1] = vel_body->y;
  vel_body_array[2] = vel_body->z;

  rates_array[0] = rates->p;
  rates_array[1] = rates->q;
  rates_array[2] = rates->r;

  float moments_array[3];
  cyclone_obm_moments(rates_array, vel_body_array, actuator_state, obm_coefficients.data, moments_array);
  struct FloatVect3 moments;
  moments.x = moments_array[0];
  moments.y = moments_array[1];
  moments.z = moments_array[2];

  return moments;
}


void evaluate_obm_f_stb_u(float fu_mat[ANDI_NUM_ACT * ANDI_OUTPUTS], const struct FloatRates *rates,
                          const struct FloatVect3 *vel_body, const float actuator_state[ANDI_NUM_ACT])
{
  float vel_body_array[3];
  (void)rates;

  vel_body_array[0] = vel_body->x;
  vel_body_array[1] = vel_body->y;
  vel_body_array[2] = vel_body->z;

  // Bound min motor speed in actuator_state to prevent really low control effectiveness which may result in instabilities.
  // This should make "free fall" more stable at the cost of some model inaccuracy at very low thrust.
  // FIXME: Rethink this solution.
  // FIXME: Apply this bounding directly on the elevon effectiveness terms in the final matrix instead of modifying actuator_state.
  // FIXME: Would it be possible to dynamically identify or saturate the control effectiveness?
  float actuator_state_bounded[ANDI_NUM_ACT];
  actuator_state_bounded[0] = actuator_state[0];
  actuator_state_bounded[1] = actuator_state[1];
  actuator_state_bounded[2] = fmaxf(actuator_state[2], 360000.0f);
  actuator_state_bounded[3] = fmaxf(actuator_state[3], 360000.0f);

  cyclone_f_stb_u(vel_body_array, actuator_state_bounded, obm_coefficients.data, fu_mat);

  fu_mat = ce_mat_tmp.data; // FIXME: Temporary hack to use ce_mat instead of the real computed matrix
}

void evaluate_obm_f_stb_x(float nu_obm[ANDI_OUTPUTS], const struct FloatRates *rates,
                          const struct FloatVect3 *vel_body, const struct FloatVect3 *ang_accel, const struct FloatVect3 *accel_body,
                          const float actuator_state[ANDI_NUM_ACT])
{
  float rates_array[3];
  float ang_accel_array[3];
  float vel_body_array[3];
  float accel_body_array[3];

  rates_array[0] = rates->p;
  rates_array[1] = rates->q;
  rates_array[2] = rates->r;

  ang_accel_array[0] = ang_accel->x;
  ang_accel_array[1] = ang_accel->y;
  ang_accel_array[2] = ang_accel->z;

  vel_body_array[0] = vel_body->x;
  vel_body_array[1] = vel_body->y;
  vel_body_array[2] = vel_body->z;

  accel_body_array[0] = accel_body->x;
  accel_body_array[1] = accel_body->y;
  accel_body_array[2] = accel_body->z;


  cyclone_f_stb_x(rates_array, ang_accel_array, vel_body_array, accel_body_array, actuator_state, obm_coefficients.data,
                  nu_obm);
}

float evaluate_obm_thrust_z(const float actuator_state[ANDI_NUM_ACT])
{
  return obm_coefficients.fx_motor_squared * (actuator_state[2] + actuator_state[3]);
}