/*
 * Copyright (C) 2016 Johan Maurin, Gautier Hattenberger
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file "modules/meteo/wind_estimator.c"
 *
 * Original Simulink files available at https://github.com/enacuavlab/UKF_Wind_Estimation
 */

#include "modules/meteo/wind_estimator.h"
#include "modules/meteo/lib_ukf_wind_estimator/UKF_Wind_Estimator.h"
#include "mcu_periph/sys_time.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"
#include "generated/modules.h"
#include "state.h"
#include <string.h>
#ifndef SITL // no chibios threads in sim
#if !USE_CHIBIOS_RTOS
#error Only Chibios is supported
#endif
#include <ch.h>
#include <hal.h>
#endif

/**
 * Default parameters
 */
#ifndef WE_UKF_KI
#define WE_UKF_KI 0.f           // >= 0 to ensure that covariance is positive semi-definite
#endif
#ifndef WE_UKF_ALPHA
#define WE_UKF_ALPHA 0.5f       // sigma point dispersion
#endif
#ifndef WE_UKF_BETA
#define WE_UKF_BETA 2.f         // 2 is the best choice when distribution is gaussian
#endif
#ifndef WE_UKF_P0
#define WE_UKF_P0 0.2f          // initial covariance diagonal element
#endif
#ifndef WE_UKF_R_GS
#define WE_UKF_R_GS 0.5f        // ground speed measurement confidence
#endif
#ifndef WE_UKF_R_VA
#define WE_UKF_R_VA 0.5f        // airspeed measurement confidence
#endif
#ifndef WE_UKF_R_AOA
#define WE_UKF_R_AOA 0.002f     // angle of attack measurement confidence
#endif
#ifndef WE_UKF_R_SSA
#define WE_UKF_R_SSA 0.002f     // sideslip angle measurement confidence
#endif
#ifndef WE_UKF_Q_VA
#define WE_UKF_Q_VA 0.1f        // airspeed model confidence
#endif
#ifndef WE_UKF_Q_VA_SCALE
#define WE_UKF_Q_VA_SCALE 0.0001f  // airspeed scale factor model confidence
#endif
#ifndef WE_UKF_Q_WIND
#define WE_UKF_Q_WIND 0.001f    // wind model confidence
#endif

#ifndef SEND_WIND_ESTIMATOR
#define SEND_WIND_ESTIMATOR TRUE
#endif

#include "modules/datalink/downlink.h"

static void send_wind_estimator(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t flags = 7; // send all data
  float upwind = -wind_estimator.wind.z;
  float airspeed = float_vect3_norm(&wind_estimator.airspeed);
  pprz_msg_send_WIND_INFO_RET(trans, dev, AC_ID,
      &flags,
      &wind_estimator.wind.y, // east
      &wind_estimator.wind.x, // north
      &upwind,
      &airspeed);
}

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

#ifndef LOG_WIND_ESTIMATOR
#define LOG_WIND_ESTIMATOR FALSE
#endif

#if LOG_WIND_ESTIMATOR
#ifndef SITL
#include "modules/loggers/sdlog_chibios.h"
#define PrintLog sdLogWriteLog
#define LogFileIsOpen() (pprzLogFile != -1)
#else // SITL: print in a file
#include <stdio.h>
#define PrintLog fprintf
#define LogFileIsOpen() (pprzLogFile != NULL)
static FILE* pprzLogFile = NULL;
#endif
static bool log_we_started;
#endif

// matrix element
#define MAT_EL(_m, _l, _c, _n) _m[_l + _c * _n]

// wind estimator public structure
struct WindEstimator wind_estimator;

// local variables
static uint32_t time_step_before;     // last periodic time

/* Thread declaration
 * MATLAB UKF is using at least 6.6KB of stack
 */
#ifndef SITL
static THD_WORKING_AREA(wa_thd_windestimation, 8 * 1024);
static __attribute__((noreturn)) void thd_windestimate(void *arg);

static MUTEX_DECL(we_ukf_mtx);        // mutex for data acces protection
static SEMAPHORE_DECL(we_ukf_sem, 0); // semaaphore for sending signal
#endif

/*----------------init_calculator---------------------*/
/* Init fonction to init different type of variable   */
/*  for the calculator before to calculate            */
/*----------------------------------------------------*/
void init_calculator(void)
{
  UKF_Wind_Estimator_initialize();

  // FIXME would be better to force Matlab to do this in initialize function
  // zero input vector
  memset(&ukf_U, 0, sizeof(ExtU));
  // zero output vector
  memset(&ukf_Y, 0, sizeof(ExtY));
  // zero internal structure
  memset(&ukf_DW, 0, sizeof(DW));
  // zero init structure
  memset(&ukf_init, 0, sizeof(ukf_init_type));
  // zero params structure
  memset(&ukf_params, 0, sizeof(ukf_params_type));

  ukf_init.x0[6] = 1.0f; // initial airspeed scale factor

  MAT_EL(ukf_init.P0, 0, 0, 7) = WE_UKF_P0;
  MAT_EL(ukf_init.P0, 1, 1, 7) = WE_UKF_P0;
  MAT_EL(ukf_init.P0, 2, 2, 7) = WE_UKF_P0;
  MAT_EL(ukf_init.P0, 3, 3, 7) = WE_UKF_P0;
  MAT_EL(ukf_init.P0, 4, 4, 7) = WE_UKF_P0;
  MAT_EL(ukf_init.P0, 5, 5, 7) = WE_UKF_P0;
  MAT_EL(ukf_init.P0, 6, 6, 7) = WE_UKF_P0;

  MAT_EL(ukf_params.R, 0, 0, 6) = powf(WE_UKF_R_GS, 2);
  MAT_EL(ukf_params.R, 1, 1, 6) = powf(WE_UKF_R_GS, 2);
  MAT_EL(ukf_params.R, 2, 2, 6) = powf(WE_UKF_R_GS, 2);
  MAT_EL(ukf_params.R, 3, 3, 6) = powf(WE_UKF_R_VA, 2);
  MAT_EL(ukf_params.R, 4, 4, 6) = powf(WE_UKF_R_AOA, 2);
  MAT_EL(ukf_params.R, 5, 5, 6) = powf(WE_UKF_R_SSA, 2);

  MAT_EL(ukf_params.Q, 0, 0, 7) = powf(WE_UKF_Q_VA, 2);
  MAT_EL(ukf_params.Q, 1, 1, 7) = powf(WE_UKF_Q_VA, 2);
  MAT_EL(ukf_params.Q, 2, 2, 7) = powf(WE_UKF_Q_VA, 2);
  MAT_EL(ukf_params.Q, 3, 3, 7) = powf(WE_UKF_Q_WIND, 2);
  MAT_EL(ukf_params.Q, 4, 4, 7) = powf(WE_UKF_Q_WIND, 2);
  MAT_EL(ukf_params.Q, 5, 5, 7) = powf(WE_UKF_Q_WIND, 2);
  MAT_EL(ukf_params.Q, 6, 6, 7) = powf(WE_UKF_Q_VA_SCALE, 2);

  ukf_init.ki = WE_UKF_KI;
  ukf_init.alpha = WE_UKF_ALPHA;
  ukf_init.beta = WE_UKF_BETA;
  ukf_params.dt = WIND_ESTIMATOR_PERIODIC_PERIOD; // actually measured later

  wind_estimator.data_available = false;
  wind_estimator.reset = false;

  wind_estimator.r_gs = WE_UKF_R_GS;
  wind_estimator.r_va = WE_UKF_R_VA;
  wind_estimator.r_aoa = WE_UKF_R_AOA;
  wind_estimator.r_ssa = WE_UKF_R_SSA;
  wind_estimator.q_va = WE_UKF_Q_VA;
  wind_estimator.q_wind = WE_UKF_Q_WIND;
  wind_estimator.q_va_scale = WE_UKF_Q_VA_SCALE;

  time_step_before = 0;
}

// Run one step and save result
static inline void wind_estimator_step(void)
{
#if LOG_WIND_ESTIMATOR
  if (LogFileIsOpen()) {
    if (!log_we_started) {
      // print header with initial parameters
      int i;
      PrintLog(pprzLogFile, "# Wind Estimator\n#\n");
      PrintLog(pprzLogFile, "# Q = diag( ");
      for (i = 0; i < 7; i++)
        PrintLog(pprzLogFile, "%.8f ", MAT_EL(ukf_params.Q, i, i, 7));
      PrintLog(pprzLogFile, ")\n");
      PrintLog(pprzLogFile, "# R = diag( ");
      for (i = 0; i < 5; i++)
        PrintLog(pprzLogFile, "%.8f ", MAT_EL(ukf_params.R, i, i, 5));
      PrintLog(pprzLogFile, ")\n");
      PrintLog(pprzLogFile, "# ki = %.5f\n", ukf_init.ki);
      PrintLog(pprzLogFile, "# alpha = %.5f\n", ukf_init.alpha);
      PrintLog(pprzLogFile, "# beta = %.5f\n", ukf_init.beta);
      PrintLog(pprzLogFile, "#\n");
      PrintLog(pprzLogFile, "p q r ax ay az q1 q2 q3 q4 vkx vky vkz va aoa ssa u v w wx wy wz vas t\n");
      log_we_started = true;
    }
    PrintLog(pprzLogFile, "%.5f %.5f %.5f %.3f %.3f %.3f %.4f %.4f %.4f %.4f %.5f %.5f %.5f %.5f %.5f %.5f ",
        ukf_U.rates[0],
        ukf_U.rates[1],
        ukf_U.rates[2],
        ukf_U.accel[0],
        ukf_U.accel[1],
        ukf_U.accel[2],
        ukf_U.q[0],
        ukf_U.q[1],
        ukf_U.q[2],
        ukf_U.q[3],
        ukf_U.vk[0],
        ukf_U.vk[1],
        ukf_U.vk[2],
        ukf_U.va,
        ukf_U.aoa,
        ukf_U.sideslip
        );
  }
#endif

  // estimate wind if airspeed is high enough
  if (ukf_U.va > 5.0f) {
    // run estimation
    UKF_Wind_Estimator_step();
    // update output structure
    wind_estimator.airspeed.x = ukf_Y.xout[0];
    wind_estimator.airspeed.y = ukf_Y.xout[1];
    wind_estimator.airspeed.z = ukf_Y.xout[2];
    wind_estimator.wind.x = ukf_Y.xout[3];
    wind_estimator.wind.y = ukf_Y.xout[4];
    wind_estimator.wind.z = ukf_Y.xout[5];
    // set ready flag
    wind_estimator.data_available = true;
  } else {
    wind_estimator.airspeed.x = 0.f;
    wind_estimator.airspeed.y = 0.f;
    wind_estimator.airspeed.z = 0.f;
    wind_estimator.wind.x = 0.f;
    wind_estimator.wind.y = 0.f;
    wind_estimator.wind.z = 0.f;
  }

#if LOG_WIND_ESTIMATOR
  if (log_we_started) {
    PrintLog(pprzLogFile, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %d\n",
        wind_estimator.airspeed.x,
        wind_estimator.airspeed.y,
        wind_estimator.airspeed.z,
        wind_estimator.wind.x,
        wind_estimator.wind.y,
        wind_estimator.wind.z,
        ukf_Y.xout[6],
        time_step_before
        );
  }
#endif
}

#include "modules/imu/imu.h"
/*----------------wind_estimator_periodic-------------*/
/*  Put Data from State in struct use by the Thread   */
/*----------------------------------------------------*/
void wind_estimator_periodic(void)
{
  // try to lock mutex without blocking
  // if it fails, it means that previous computation took too long
#ifndef SITL
  if (chMtxTryLock(&we_ukf_mtx)) {
#endif
    if (wind_estimator.reset) {
      init_calculator();
    }
    // update input vector from state interface
    ukf_U.rates[0] = stateGetBodyRates_f()->p;  // rad/s
    ukf_U.rates[1] = stateGetBodyRates_f()->q;  // rad/s
    ukf_U.rates[2] = stateGetBodyRates_f()->r;  // rad/s
    // transform data in body frame
    struct FloatVect3 accel_ned = {
      stateGetAccelNed_f()->x,
      stateGetAccelNed_f()->y,
      stateGetAccelNed_f()->z
    };
//    struct FloatRMat *ned_to_body = stateGetNedToBodyRMat_f();
//    struct FloatVect3 accel_body;
//    float_rmat_vmult(&accel_body, ned_to_body, &accel_ned);

    ///// IMU test
    struct FloatVect3 accel_body = {
      .x = ACCEL_FLOAT_OF_BFP(imu.accel.x),
      .y = ACCEL_FLOAT_OF_BFP(imu.accel.y),
      .z = ACCEL_FLOAT_OF_BFP(imu.accel.z)
    };
    struct FloatVect3 tmp = accel_body;
    //struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&body_to_imu);
    //int32_rmat_transp_vmult(&accel_body, body_to_imu_rmat, accel);
    // tilt and remove gravity
    struct FloatVect3 gravity_ned = { 0.f, 0.f, 9.81f };
    struct FloatVect3 gravity_body = { 0.f, 0.f, 0.f };
#if 0
    struct FloatRMat *ned_to_body_rmat = stateGetNedToBodyRMat_f();
    float_rmat_vmult(&gravity_body, ned_to_body_rmat, &gravity_ned);
    VECT3_ADD(accel_body, gravity_body);
#else
    struct FloatEulers *b2i_e = stateGetNedToBodyEulers_f();
    accel_body.x += -9.81*sinf(b2i_e->theta);
    accel_body.y += 9.81*cosf(b2i_e->theta)*sinf(b2i_e->phi);
    accel_body.z += 9.81*cosf(b2i_e->theta)*cosf(b2i_e->phi);
#endif
    ///// End test

    ukf_U.accel[0] = accel_body.x;   // m/s^2
    ukf_U.accel[1] = accel_body.y;   // m/s^2
    ukf_U.accel[2] = accel_body.z;   // m/s^2
    ukf_U.q[0] = stateGetNedToBodyQuat_f()->qi;
    ukf_U.q[1] = stateGetNedToBodyQuat_f()->qx;
    ukf_U.q[2] = stateGetNedToBodyQuat_f()->qy;
    ukf_U.q[3] = stateGetNedToBodyQuat_f()->qz;
    ukf_U.vk[0] = stateGetSpeedNed_f()->x;      // m/s
    ukf_U.vk[1] = stateGetSpeedNed_f()->y;      // m/s
    ukf_U.vk[2] = stateGetSpeedNed_f()->z;      // m/s
    ukf_U.va = stateGetAirspeed_f();            // m/s
    ukf_U.aoa = stateGetAngleOfAttack_f();      // rad.
    ukf_U.sideslip = stateGetSideslip_f();      // rad.

    ///// TEST
    // update input vector from state interface
    //ukf_U.rates[0] = 0.;
    //ukf_U.rates[1] = 0.;
    //ukf_U.rates[2] = 0.;
    //ukf_U.accel[0] = 0.;
    //ukf_U.accel[1] = 0.;
    //ukf_U.accel[2] = 0.;
    //ukf_U.q[0] = 1.0;
    //ukf_U.q[1] = 0.;
    //ukf_U.q[2] = 0.;
    //ukf_U.q[3] = 0.;
    //ukf_U.vk[0] = 15.;
    //ukf_U.vk[1] = 0.;
    //ukf_U.vk[2] = 0.;
    //ukf_U.va = 15.;
    //ukf_U.aoa = 0.;
    //ukf_U.sideslip = 0.;

    //DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 16, (float *)(&ukf_U));

    float msg[] = {
      tmp.x,
      tmp.y,
      tmp.z,
      gravity_body.x,
      gravity_body.y,
      gravity_body.z,
      accel_body.x,
      accel_body.y,
      accel_body.z,
      accel_ned.x,
      accel_ned.y,
      accel_ned.z
    };
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 12, msg);

    // compute DT and set input vector
    if (time_step_before == 0) {
      ukf_params.dt = WIND_ESTIMATOR_PERIODIC_PERIOD;
      time_step_before = get_sys_time_msec();
    } else {
      ukf_params.dt = (get_sys_time_msec() - time_step_before) / 1000.f;
      time_step_before = get_sys_time_msec();
    }
#ifndef SITL
    chMtxUnlock(&we_ukf_mtx);
    // send signal to computation thread
    chSemSignal(&we_ukf_sem);
  }
#else
  wind_estimator_step();
#endif
}

/*----------------- wind_estimator_event -------------*/
/*  Fonction put data in State                        */
/*----------------------------------------------------*/
void wind_estimator_event(void)
{
  if (wind_estimator.data_available) {
#ifndef SITL
    if (chMtxTryLock(&we_ukf_mtx)) {
#endif
      struct FloatVect2 wind_ne = {
        wind_estimator.wind.x,
        wind_estimator.wind.y
      };
      stateSetHorizontalWindspeed_f(&wind_ne);            //NEED CHECK
      stateSetVerticalWindspeed_f(wind_estimator.wind.z); //NEED CHECK

      // TODO do something with corrected airspeed norm

#if SEND_WIND_ESTIMATOR
      // send over telemetry
      send_wind_estimator(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif

      wind_estimator.data_available = false;
#ifndef SITL
      chMtxUnlock(&we_ukf_mtx);
    }
#endif
  }
}

/*------------------thd_windestimate------------------*/
/*  Thread fonction                                   */
/*----------------------------------------------------*/
#ifndef SITL
static void thd_windestimate(void *arg)
{
  (void) arg;
  chRegSetThreadName("wind estimation");

  while (true) {
    // wait for incoming request signal
    chSemWait(&we_ukf_sem);
    // lock state
    chMtxLock(&we_ukf_mtx);
    // run estimation step
    wind_estimator_step();
    // unlock
    chMtxUnlock(&we_ukf_mtx);
  }
}
#endif

/*-----------------wind_estimator_init----------------*/
/*  Init the Thread and the calculator                */
/*----------------------------------------------------*/
void wind_estimator_init(void)
{
  init_calculator();

#if LOG_WIND_ESTIMATOR
  log_we_started = false;
#if SITL
  // open log file for writing
  // path should be specified in airframe file
  uint32_t counter = 0;
  char filename[512];
  snprintf(filename, 512, "%s/we_ukf_%05d.csv", STRINGIFY(WE_LOG_PATH), counter);
  // check availale name
  while ((pprzLogFile = fopen(filename, "r"))) {
    fclose(pprzLogFile);
    snprintf(filename, 512, "%s/we_ukf_%05d.csv", STRINGIFY(WE_LOG_PATH), ++counter);
  }
  pprzLogFile = fopen(filename, "w");
  if (pprzLogFile == NULL) {
    printf("Failed to open WE log file '%s'\n",filename);
  } else {
    printf("Opening WE log file '%s'\n",filename);
  }
#endif
#endif

#ifndef SITL
  // Start wind estimation thread
  chThdCreateStatic(wa_thd_windestimation, sizeof(wa_thd_windestimation),
                    NORMALPRIO + 2, thd_windestimate, NULL);
#endif

#if PERIODIC_TELEMETRY
  // register for periodic telemetry (for logging with flight recorder)
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WIND_INFO_RET, send_wind_estimator);
#endif
}

void wind_estimator_Set_R_GS(float _v)
{
  wind_estimator.r_gs = _v;
  MAT_EL(ukf_params.R, 0, 0, 6) = powf(_v, 2);
  MAT_EL(ukf_params.R, 1, 1, 6) = powf(_v, 2);
  MAT_EL(ukf_params.R, 2, 2, 6) = powf(_v, 2);
}

void wind_estimator_Set_R_VA(float _v)
{
  wind_estimator.r_va = _v;
  MAT_EL(ukf_params.R, 3, 3, 6) = powf(_v, 2);
}

void wind_estimator_Set_R_AOA(float _v)
{
  wind_estimator.r_aoa = _v;
  MAT_EL(ukf_params.R, 4, 4, 6) = powf(_v, 2);
}

void wind_estimator_Set_R_SSA(float _v)
{
  wind_estimator.r_ssa = _v;
  MAT_EL(ukf_params.R, 5, 5, 6) = powf(_v, 2);
}

void wind_estimator_Set_Q_VA(float _v)
{
  wind_estimator.q_va = _v;
  MAT_EL(ukf_params.Q, 0, 0, 7) = powf(_v, 2);
  MAT_EL(ukf_params.Q, 1, 1, 7) = powf(_v, 2);
  MAT_EL(ukf_params.Q, 2, 2, 7) = powf(_v, 2);
}

void wind_estimator_Set_Q_WIND(float _v)
{
  wind_estimator.q_wind = _v;
  MAT_EL(ukf_params.Q, 3, 3, 7) = powf(_v, 2);
  MAT_EL(ukf_params.Q, 4, 4, 7) = powf(_v, 2);
  MAT_EL(ukf_params.Q, 5, 5, 7) = powf(_v, 2);
}

void wind_estimator_Set_Q_VA_SCALE(float _v)
{
  wind_estimator.q_va_scale = _v;
  MAT_EL(ukf_params.Q, 6, 6, 7) = powf(_v, 2);
}

