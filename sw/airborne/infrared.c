/*
 * Paparazzi mcu0 $Id$
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */
/** \file infrared.c
 *  \brief Regroup all functions link to \a ir
 */

#include <stdlib.h>

#include "adc.h"
#include "infrared.h"
#include "gps.h"
#include "autopilot.h"
#include "estimator.h"
#include "ap_downlink.h"
#include "sys_time.h"
#include "airframe.h"

int16_t ir_roll;
int16_t ir_pitch;
int16_t ir_top;

float z_contrast_mode;

float ir_roll_neutral;
float ir_pitch_neutral;

bool_t ir_360;
float ir_estimated_phi_pi_4, ir_estimated_phi_minus_pi_4;
float ir_estimated_theta_pi_4, ir_estimated_theta_minus_pi_4;

#ifndef IR_ESTIMATED_PHI_PI_4
#define IR_ESTIMATED_PHI_PI_4 M_PI_4
#endif

#ifndef IR_ESTIMATED_PHI_MINUS_PI_4
#define IR_ESTIMATED_PHI_MINUS_PI_4 IR_ESTIMATED_PHI_PI_4
#endif

#ifndef IR_ESTIMATED_THETA_PI_4
#define IR_ESTIMATED_THETA_PI_4 IR_ESTIMATED_PHI_PI_4
#endif

#ifndef IR_ESTIMATED_THETA_MINUS_PI_4
#define IR_ESTIMATED_THETA_MINUS_PI_4 IR_ESTIMATED_THETA_PI_4
#endif


#if defined IR_CORRECTION_LEFT && defined IR_CORRECTION_RIGHT
float ir_correction_left;
float ir_correction_right;
#endif

#if defined IR_CORRECTION_UP && defined IR_CORRECTION_DOWN
float ir_correction_up;
float ir_correction_down;
#endif

/** Initialized to \a IR_DEFAULT_CONTRAST. Changed with calibration */
int16_t ir_contrast;
/** Initialized to \a IR_DEFAULT_CONTRAST.
 *  Changed with @@@@@ EST-CE QUE CA CHANGE @@@@@ */

/** \def RadOfIrFromContrast(c)
 *  \brief Contrast measurement
 *  \note <b>Plane must be nose down !</b>
 */
#define RadOfIrFromContrast(c) ir_rad_of_ir = IR_RAD_OF_IR_CONTRAST / c;

/** rad_of_ir variable factor: let convert \a ir value in radian.
 *  Initialized with airframe \a IR_RAD_OF_IR_CONTRAST and \a IR_DEFAULT_CONTRAST constants. \n
 *  Change when \a lls work.
 */
float ir_rad_of_ir;

float estimator_rad_of_ir, estimator_ir, estimator_rad;

#ifndef SITL
static struct adc_buf buf_ir1;
static struct adc_buf buf_ir2;
#endif

#ifdef ADC_CHANNEL_IR_TOP
static struct adc_buf buf_ir_top;
#endif

#ifndef ADC_CHANNEL_IR_NB_SAMPLES
#define ADC_CHANNEL_IR_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

#ifndef Z_CONTRAST_DEFAULT
#define Z_CONTRAST_DEFAULT 1
#endif
#ifdef Z_CONTRAST_START
#warning "Z_CONTRAST_START deprecated !! z_contrast mode now defaults to Z_CONTRAST_DEFAULT or 1"
#endif

float ir_360_lateral_correction;
float ir_360_longitudinal_correction;
float ir_360_vertical_correction;


#ifndef IR_360_LATERAL_CORRECTION
#define IR_360_LATERAL_CORRECTION 1.
#endif

#ifndef IR_360_LONGITUDINAL_CORRECTION
#define IR_360_LONGITUDINAL_CORRECTION 1.
#endif

#ifndef IR_360_VERTICAL_CORRECTION
#define IR_360_VERTICAL_CORRECTION 1.
#endif



/** \brief Initialisation of \a ir */
/** Initialize \a ir with the \a IR_DEFAULT_CONTRAST \n
 *  Initialize \a adc_buf_channel
 */
void ir_init(void) {
#ifndef SITL
  adc_buf_channel(ADC_CHANNEL_IR1, &buf_ir1, ADC_CHANNEL_IR_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_IR2, &buf_ir2, ADC_CHANNEL_IR_NB_SAMPLES);
#endif

#ifdef ADC_CHANNEL_IR_TOP
  adc_buf_channel(ADC_CHANNEL_IR_TOP, &buf_ir_top, ADC_CHANNEL_IR_NB_SAMPLES);
  z_contrast_mode = Z_CONTRAST_DEFAULT;
#else
  z_contrast_mode = 0;
  RadOfIrFromContrast(IR_DEFAULT_CONTRAST);
#endif
 
  ir_roll_neutral  = RadOfDeg(IR_ROLL_NEUTRAL_DEFAULT);
  ir_pitch_neutral = RadOfDeg(IR_PITCH_NEUTRAL_DEFAULT);

  estimator_rad_of_ir = ir_rad_of_ir;

#if defined IR_CORRECTION_LEFT && defined IR_CORRECTION_RIGHT
  ir_correction_left = IR_CORRECTION_LEFT;
  ir_correction_right = IR_CORRECTION_RIGHT;
#endif

#if defined IR_CORRECTION_UP && defined IR_CORRECTION_DOWN
  ir_correction_up = IR_CORRECTION_UP;
  ir_correction_down = IR_CORRECTION_DOWN;
#endif

  ir_360 = TRUE;
  ir_estimated_phi_pi_4 = IR_ESTIMATED_PHI_PI_4;
  ir_estimated_phi_minus_pi_4 = IR_ESTIMATED_PHI_MINUS_PI_4;
  ir_estimated_theta_pi_4 = IR_ESTIMATED_THETA_PI_4;
  ir_estimated_theta_minus_pi_4 = IR_ESTIMATED_THETA_MINUS_PI_4;

  ir_360_lateral_correction = IR_360_LATERAL_CORRECTION;
  ir_360_longitudinal_correction = IR_360_LONGITUDINAL_CORRECTION;
  ir_360_vertical_correction = IR_360_VERTICAL_CORRECTION;

#ifndef ADC_CHANNEL_IR_TOP
  ir_contrast = IR_DEFAULT_CONTRAST;
  ir_rad_of_ir = IR_RAD_OF_IR_CONTRAST / IR_DEFAULT_CONTRAST;
#endif
}

/** \brief Update \a ir_roll and ir_pitch from ADCs or from simulator
 * message in HITL and SITL modes
 */
void ir_update(void) {
#if ! (defined SITL || defined HITL)
  int16_t x1_mean = buf_ir1.sum/buf_ir1.av_nb_sample;
  int16_t x2_mean = buf_ir2.sum/buf_ir2.av_nb_sample;
  ir_roll = IR_RollOfIrs(x1_mean, x2_mean);
  ir_pitch = IR_PitchOfIrs(x1_mean, x2_mean);
#ifdef ADC_CHANNEL_IR_TOP
  ir_top =  IR_TopOfIr(buf_ir_top.sum/buf_ir_top.av_nb_sample - IR_ADC_TOP_NEUTRAL);
#endif

  /** neutrals are not taken into account in SITL and HITL */
  ir_roll -= IR_ADC_ROLL_NEUTRAL;
  ir_pitch -= IR_ADC_PITCH_NEUTRAL;
#endif /* !SITL && !HITL */
/** #else ir_roll set by simulator in sim_ir.c */
}


uint8_t calib_status = NO_CALIB;

#ifndef ADC_CHANNEL_IR_TOP
/** \brief Contrast measurement
 *  \note <b>Plane must be nose down !</b>
 */
static void ir_gain_calib(void) {
  /* plane nose down -> negativ value */
  ir_contrast = abs(ir_pitch);
  RadOfIrFromContrast(ir_contrast);
}

/** Maximal delay waits before calibration.
		After, no more calibration is possible */
#define MAX_DELAY_FOR_CALIBRATION 10

/** \brief Calibrate contrast if paparazzi mode is
 * set to auto1 before MAX_DELAY_FOR_CALIBRATION secondes */
/**User must put verticaly the uav (nose bottom) and push
 * radio roll stick to get new calibration
 * If not, the default calibration is used.
 */
void ground_calibrate( bool_t triggered ) {
  switch (calib_status) {
  case NO_CALIB:
    if (cpu_time_sec < MAX_DELAY_FOR_CALIBRATION && pprz_mode == PPRZ_MODE_AUTO1 ) {
      calib_status = WAITING_CALIB_CONTRAST;
      DOWNLINK_SEND_CALIB_START();
    }
    break;
  case WAITING_CALIB_CONTRAST:
    if (triggered) {
      ir_gain_calib();
      estimator_rad_of_ir = ir_rad_of_ir;
      calib_status = CALIB_DONE;
      DOWNLINK_SEND_CALIB_CONTRAST(&ir_contrast);
    }
    break;
  case CALIB_DONE:
    break;
  }
}
#endif /* !ADC_CHANNEL_IR_TOP */

#define INIT_WEIGHT 100. /* The number of times the initial value has to be taken */
#define RHO 0.995 /* The higher, the slower the estimation is changing */

#define G 9.81

void estimator_update_ir_estim( void ) {
  static float last_hspeed_dir;
  static uint32_t last_t; /* ms */
  static bool_t initialized = FALSE;
  static float sum_xy, sum_xx;

  if (initialized) {
    float dt = (float)(gps_itow - last_t) / 1e3;
    if (dt > 0.1) { // Against division by zero
      float dpsi = (estimator_hspeed_dir - last_hspeed_dir); 
      NormRadAngle(dpsi);
      estimator_rad = dpsi/dt*NOMINAL_AIRSPEED/G; /* tan linearized */
      NormRadAngle(estimator_rad);
      estimator_ir = (float)ir_roll;
      float absphi = fabs(estimator_rad);
      if (absphi < 1.0 && absphi > 0.05 && (- ir_contrast/2 < ir_roll && ir_roll < ir_contrast/2)) {
	sum_xy = estimator_rad * estimator_ir + RHO * sum_xy;
	sum_xx = estimator_ir * estimator_ir + RHO * sum_xx;
#if defined IR_RAD_OF_IR_MIN_VALUE & defined IR_RAD_OF_IR_MAX_VALUE
	float result = sum_xy / sum_xx;
	if (result < IR_RAD_OF_IR_MIN_VALUE)
	  estimator_rad_of_ir = IR_RAD_OF_IR_MIN_VALUE;
	else if (result > IR_RAD_OF_IR_MAX_VALUE)
	  estimator_rad_of_ir = IR_RAD_OF_IR_MAX_VALUE;
	else
	  estimator_rad_of_ir = result;
#else
	  estimator_rad_of_ir = sum_xy / sum_xx;
#endif
      }
    } 
  } else {
    initialized = TRUE;
    float init_ir2 = ir_contrast;
    init_ir2 = init_ir2*init_ir2;
    sum_xy = INIT_WEIGHT * estimator_rad_of_ir * init_ir2;
    sum_xx = INIT_WEIGHT * init_ir2;
  }

  last_hspeed_dir = estimator_hspeed_dir;
  last_t = gps_itow;
}


/* Correction of the infrared estimation roll angle: returns pi/4 for
   est_pi_4 */
static inline float correct_angle(float m_angle, float est_pi_4, float est_minus_pi_4) {
  if (m_angle >= 0 && m_angle < est_pi_4) 
    /* 0 .. est_pi_4 */
    return (m_angle * M_PI_4 / est_pi_4);
  else if (m_angle > M_PI - est_pi_4)
    /* pi-est_pi_4 .. pi */
    return (m_angle - (M_PI - est_pi_4))* M_PI_4 / est_pi_4 + 3*M_PI_4;
  else if (m_angle >= est_pi_4)
    /* est_pi_4 .. pi-est_pi_4 */
    return (m_angle - est_pi_4) * M_PI_4 / (M_PI_2 - est_pi_4) + M_PI_4;
  else if (m_angle <= 0 && m_angle > -est_minus_pi_4) 
    /* -est_pi_4 .. 0 */
    return (m_angle * M_PI_4 / est_minus_pi_4);
  else if (m_angle < - (M_PI - est_minus_pi_4))
    /* -pi .. -(pi-est_pi_4) */
    return (m_angle + M_PI)* M_PI_4 / est_minus_pi_4 + -M_PI;
  else
    /* -(pi-est_pi_4) .. -est_pi_4 */
    return (m_angle - (-M_PI+est_minus_pi_4)) * M_PI_4 / (M_PI_2 - est_minus_pi_4) - 3 * M_PI_4;
}


static inline void ir_correction( void ) {
    /* infrared compensation */
#if defined IR_CORRECTION_LEFT && defined IR_CORRECTION_RIGHT
    if (estimator_phi >= 0) 
      estimator_phi *= ir_correction_right;
    else
      estimator_phi *= ir_correction_left;
#endif
    
    
#if defined IR_CORRECTION_UP && defined IR_CORRECTION_DOWN
    if (estimator_theta >= 0)
      estimator_theta *= ir_correction_up;
    else
      estimator_theta *= ir_correction_down;
#endif
}

void estimator_update_state_infrared( void ) {
  float rad_of_ir = (ir_estim_mode == IR_ESTIM_MODE_ON ? 
		     estimator_rad_of_ir :
		     ir_rad_of_ir);

#ifdef IR_360
  if (ir_360) { /* 360° estimation */
    /* 250 us for the whole block */
    float tmp_ir_roll = ir_roll * ir_360_lateral_correction;
    float tmp_ir_pitch = ir_pitch * ir_360_longitudinal_correction;
    float tmp_ir_top = ir_top * ir_360_vertical_correction;

    estimator_phi  = atan2(tmp_ir_roll, tmp_ir_top) - ir_roll_neutral;
    estimator_phi = correct_angle(estimator_phi, ir_estimated_phi_pi_4, ir_estimated_phi_minus_pi_4);

    estimator_theta  = atan2(tmp_ir_pitch, tmp_ir_top) - ir_pitch_neutral;
    estimator_theta = correct_angle(estimator_theta, ir_estimated_theta_pi_4, ir_estimated_theta_minus_pi_4);
    if (estimator_theta < -M_PI_2)
      estimator_theta += M_PI;
    else if (estimator_theta > M_PI_2)
      estimator_theta -= M_PI;

    ir_correction();
  } else
#endif /* IR_360 */
  {
    ir_top = Max(ir_top, 1);
    float c = rad_of_ir*(1-z_contrast_mode)+z_contrast_mode*((float)IR_RAD_OF_IR_CONTRAST/fabs(ir_top));
    estimator_phi  = c * ir_roll - ir_roll_neutral;
    estimator_theta = c * ir_pitch - ir_pitch_neutral;
    
    ir_correction();    
    
    Bound(estimator_phi, -M_PI_2, M_PI_2);
    Bound(estimator_theta, -M_PI_2, M_PI_2);
    
 } 
}
