/*
 * $Id$
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

#include <inttypes.h>
#include <math.h>


#include "link_autopilot.h"

#include "timer.h"
#include "adc.h"
#include "pid.h"
#include "gps.h"
#include "infrared.h"
#include "downlink.h"
#include "nav.h"
#include "autopilot.h"
#include "estimator.h"
#include "if_calib.h"

//
//
// FIXME estimator_flight_time should not be manipuled here anymore
//
#define MIN_SPEED_FOR_TAKEOFF 5. // m/s


uint8_t fatal_error_nb = 0;
static const uint16_t version = 1;

static uint16_t cputime = 0;     // seconds

uint8_t  pprz_mode = PPRZ_MODE_MANUAL;
uint8_t  vertical_mode = VERTICAL_MODE_MANUAL;
uint8_t  ir_estim_mode = IR_ESTIM_MODE_ON;

bool_t rc_event_1, rc_event_2;

uint8_t vsupply;

static uint8_t  mcu1_status, mcu1_ppm_cpt;

static bool_t low_battery = FALSE;

#ifdef CTL_BRD_V1_1
struct adc_buf buf_bat;
#endif

float slider_1_val, slider_2_val;

bool_t launch = FALSE;


#define Min(x, y) (x < y ? x : y)
#define Max(x, y) (x > y ? x : y)


#define NO_CALIB               0
#define WAITING_CALIB_CONTRAST 1
#define CALIB_DONE             2

#define MAX_DELAY_FOR_CALIBRATION 10


inline void ground_calibrate(void) {
  static uint8_t calib_status = NO_CALIB;
  switch (calib_status) {
  case NO_CALIB:
    if (cputime < MAX_DELAY_FOR_CALIBRATION && pprz_mode == PPRZ_MODE_AUTO1 ) {
      calib_status = WAITING_CALIB_CONTRAST;
      DOWNLINK_SEND_CALIB_START();
    }
    break;
  case WAITING_CALIB_CONTRAST:
    if (STICK_PUSHED(from_fbw.channels[RADIO_ROLL])) {
      ir_gain_calib();
      estimator_rad_of_ir = ir_rad_of_ir;
      DOWNLINK_SEND_RAD_OF_IR(&estimator_ir, &estimator_rad, &estimator_rad_of_ir, &ir_roll_neutral, &ir_pitch_neutral);
      calib_status = CALIB_DONE;
      DOWNLINK_SEND_CALIB_CONTRAST(&ir_contrast);
    }
    break;
  case CALIB_DONE:
    break;
  }
}



inline uint8_t pprz_mode_update( void ) {
  /* We remain in home mode until explicit reset from the RC */
  if (pprz_mode != PPRZ_MODE_HOME || CheckEvent(rc_event_1)) {
    ModeUpdate(pprz_mode, PPRZ_MODE_OF_PULSE(from_fbw.channels[RADIO_MODE], from_fbw.status));
    nav_stage = 0; /* Restart the last current block */
  } else
    return FALSE;
}

#ifdef RADIO_LLS
inline uint8_t ir_estim_mode_update( void ) {
  ModeUpdate(ir_estim_mode, IR_ESTIM_MODE_OF_PULSE(from_fbw.channels[RADIO_LLS]));
}
#endif


inline uint8_t mcu1_status_update( void ) {
  uint8_t new_mode = from_fbw.status;
  if (mcu1_status != new_mode) {
    bool_t changed = ((mcu1_status&MASK_FBW_CHANGED) != (new_mode&MASK_FBW_CHANGED));
    mcu1_status = new_mode;
    return changed;
  }
  return FALSE;
}

#define EVENT_DELAY 20

#define EventUpdate(_cpt, _cond, _event) \
  if (_cond) { \
    if (_cpt < EVENT_DELAY) { \
      _cpt++; \
      if (_cpt == EVENT_DELAY) \
        _event = TRUE; \
    } \
  } else { \
    _cpt = 0; \
    _event = FALSE; \
  }
#define EventPos(_cpt, _channel, _event) \
 EventUpdate(_cpt, (inflight_calib_mode==IF_CALIB_MODE_NONE && from_fbw.channels[_channel]>MAX_PPRZ/2), _event)

#define EventNeg(_cpt, _channel, _event) \
 EventUpdate(_cpt, (inflight_calib_mode==IF_CALIB_MODE_NONE && from_fbw.channels[_channel]<-MAX_PPRZ/2), _event)

static inline void events_update(void) {
  static uint16_t event1_cpt = 0;
  EventPos(event1_cpt, RADIO_GAIN1, rc_event_1);
  static uint16_t event2_cpt = 0;
  EventNeg(event2_cpt, RADIO_GAIN1, rc_event_2);
}  


/* Send back uncontrolled channels (only rudder) */
inline void copy_from_to_fbw ( void ) {
  to_fbw.channels[RADIO_YAW] = from_fbw.channels[RADIO_YAW];
  to_fbw.status = 0;
}

#ifdef EST_TEST
float est_pos_x;
float est_pos_y;
float est_fcourse;
uint8_t ticks_last_est; // 20Hz
#endif /* EST_TEST */



/* 
   called at 20Hz.
   sends a serie of initialisation messages followed by a stream of periodic ones
*/ 

#define INIT_MSG_NB 2
#define HI_FREQ_PHASE_NB  5

static char ac_ident[16] = AIRFRAME_NAME;

#define PERIODIC_SEND_BAT() DOWNLINK_SEND_BAT(&vsupply, &estimator_flight_time, &low_battery, &block_time, &stage_time)
#define PERIODIC_SEND_DEBUG() DOWNLINK_SEND_DEBUG(&link_fbw_nb_err, &link_fbw_fbw_nb_err, &modem_nb_ovrn, &gps_nb_ovrn, &mcu1_ppm_cpt);
#define PERIODIC_SEND_ATTITUDE() DOWNLINK_SEND_ATTITUDE(&estimator_phi, &estimator_psi, &estimator_theta);  
#define PERIODIC_SEND_ADC() DOWNLINK_SEND_ADC(&ir_roll, &ir_pitch);
#define PERIODIC_SEND_STABILISATION() DOWNLINK_SEND_STABILISATION(&roll_pgain, &pitch_pgain);
#define PERIODIC_SEND_CLIMB_PID() DOWNLINK_SEND_CLIMB_PID(&desired_gaz, &desired_climb, &climb_sum_err, &climb_pgain);
#define PERIODIC_SEND_PPRZ_MODE() DOWNLINK_SEND_PPRZ_MODE(&pprz_mode, &vertical_mode, &inflight_calib_mode, &mcu1_status, &ir_estim_mode);
#define PERIODIC_SEND_DESIRED() DOWNLINK_SEND_DESIRED(&desired_roll, &desired_pitch, &desired_x, &desired_y, &desired_altitude);
#define PERIODIC_SEND_PITCH() DOWNLINK_SEND_PITCH(&ir_pitch, &ir_pitch_neutral, &ir_gain);
#define PERIODIC_SEND_NAVIGATION_REF()  DOWNLINK_SEND_NAVIGATION_REF(&utm_east0, &utm_north0);
#define PERIODIC_SEND_IDENT()  DOWNLINK_SEND_IDENT(ac_ident);

#ifdef RADIO_CALIB
#define PERIODIC_SEND_SETTINGS() if (inflight_calib_mode != IF_CALIB_MODE_NONE)	DOWNLINK_SEND_SETTINGS(&inflight_calib_mode, &slider_1_val, &slider_2_val);
#else
#define PERIODIC_SEND_SETTINGS()
#endif


inline void reporting_task( void ) {
  static uint8_t boot = TRUE;

  /* initialisation phase */
  if (boot) {
      DOWNLINK_SEND_BOOT(&version);
      DOWNLINK_SEND_RAD_OF_IR(&estimator_ir, &estimator_rad, &estimator_rad_of_ir, &ir_roll_neutral, &ir_pitch_neutral);
      boot = FALSE;
  }
  /* periodic reporting */
  else {
    PeriodicSend();
  }
}

inline uint8_t inflight_calib_mode_update (void) {
  ModeUpdate(inflight_calib_mode, IF_CALIB_MODE_OF_PULSE(from_fbw.channels[RADIO_CALIB]));
}


inline void radio_control_task( void ) {
  if (link_fbw_receive_valid) {
    uint8_t mode_changed = FALSE;
    copy_from_to_fbw();
    if ((bit_is_set(from_fbw.status, RADIO_REALLY_LOST) && (pprz_mode == PPRZ_MODE_AUTO1 || pprz_mode == PPRZ_MODE_MANUAL)) || too_far_from_home) {
      pprz_mode = PPRZ_MODE_HOME;
      mode_changed = TRUE;
    }
    if (bit_is_set(from_fbw.status, AVERAGED_CHANNELS_SENT)) {
      bool_t pprz_mode_changed = pprz_mode_update();
      mode_changed |= pprz_mode_changed;
#ifdef RADIO_LLS
      mode_changed |= ir_estim_mode_update();
#endif
#ifdef RADIO_CALIB
      bool_t calib_mode_changed = inflight_calib_mode_update();
      inflight_calib(calib_mode_changed || pprz_mode_changed);
      mode_changed |= calib_mode_changed;
#endif
    }
    mode_changed |= mcu1_status_update();
    if ( mode_changed )
      DOWNLINK_SEND_PPRZ_MODE(&pprz_mode, &vertical_mode, &inflight_calib_mode, &mcu1_status, &ir_estim_mode);
      
    if (pprz_mode == PPRZ_MODE_AUTO1) {
      desired_roll = FLOAT_OF_PPRZ(from_fbw.channels[RADIO_ROLL], 0., -0.6);
      desired_pitch = FLOAT_OF_PPRZ(from_fbw.channels[RADIO_PITCH], 0., 0.5);
    } // else asynchronously set by course_pid_run()
    if (pprz_mode == PPRZ_MODE_MANUAL || pprz_mode == PPRZ_MODE_AUTO1) 
      desired_gaz = from_fbw.channels[RADIO_THROTTLE];
    // else asynchronously set by climb_pid_run();

    mcu1_ppm_cpt = from_fbw.ppm_cpt;
#ifndef CTL_BRD_V1_1
    vsupply = from_fbw.vsupply;
#endif

    events_update();

    if (!estimator_flight_time) {
      ground_calibrate();
      if (pprz_mode == PPRZ_MODE_AUTO2 && from_fbw.channels[RADIO_THROTTLE] > GAZ_THRESHOLD_TAKEOFF) {
	launch = TRUE;
      }
    }
  }

}

void navigation_task( void ) {

  /* Compute desired_course */
  if (pprz_mode == PPRZ_MODE_HOME)
    nav_home();
  else
    nav_update_desired_course();
  
  DOWNLINK_SEND_NAVIGATION(&nav_block, &nav_stage, &estimator_x, &estimator_y, &desired_course, &dist2_to_wp, &course_pgain, &dist2_to_home);
  
  if (pprz_mode == PPRZ_MODE_AUTO2 || pprz_mode == PPRZ_MODE_HOME) {
    course_pid_run(); /* aka compute desired_roll */
    desired_pitch = nav_pitch;
    
    if (vertical_mode == VERTICAL_MODE_AUTO_ALT)
      altitude_pid_run();
    if (vertical_mode >= VERTICAL_MODE_AUTO_CLIMB)
      climb_pid_run();
    if (vertical_mode == VERTICAL_MODE_AUTO_GAZ)
      desired_gaz = nav_desired_gaz;
    if (low_battery || (!estimator_flight_time && !launch))
      desired_gaz = 0.;
  }  



}

#define PERIOD (256. * 1024. / CLOCK / 1000000.)
inline void periodic_task( void ) { // 61 Hz
  static uint8_t _20Hz   = 0;
  static uint8_t _4Hz   = 0;
  static uint8_t _1Hz   = 0;

  estimator_t += PERIOD;

  _20Hz++;
  if (_20Hz>=3) _20Hz=0;
  _4Hz++;
  if (_4Hz>=15) _4Hz=0;
  _1Hz++;
  if (_1Hz>=61) _1Hz=0;
  
  if (!_1Hz) {
    if (estimator_flight_time) estimator_flight_time++;
    cputime++;
    stage_time++;
    block_time++;

#ifdef CTL_BRD_V1_1
    uint16_t av_bat_value = buf_bat.sum/AV_NB_SAMPLE;
    vsupply = VoltageOfAdc(av_bat_value) * 10.; 
#endif
    low_battery |= (vsupply < LOW_BATTERY);
  }
  switch(_4Hz) {
  case 0:
    estimator_propagate_state();
    navigation_task();
    break;
    //  default:
  }
  switch (_20Hz) {
  case 0:
    break;
  case 1: {
    static uint8_t odd;
    odd++;
    if (odd & 0x01)
      reporting_task();
    break;
  }
  case 2:
    ir_update();
    estimator_update_state_infrared();
    roll_pitch_pid_run(); // Set  desired_aileron & desired_elevator
    to_fbw.channels[RADIO_THROTTLE] = desired_gaz; // desired_gaz is set upon GPS message reception
    to_fbw.channels[RADIO_ROLL] = desired_aileron;
    to_fbw.channels[RADIO_PITCH] = desired_elevator;
    
    // Code for camera stabilization, FIXME put that elsewhere
    to_fbw.channels[RADIO_GAIN1] = TRIM_PPRZ(MAX_PPRZ/0.75*(-estimator_phi));
    
    link_fbw_send();
    break;
  default:
    fatal_error_nb++;
  }
}


void use_gps_pos(void) {
  DOWNLINK_SEND_GPS(&gps_mode, &gps_utm_east, &gps_utm_north, &gps_fcourse, &gps_falt, &gps_fspeed,&gps_fclimb, &gps_ftow);
  estimator_update_state_gps();
  DOWNLINK_SEND_RAD_OF_IR(&estimator_ir, &estimator_rad, &estimator_rad_of_ir, &ir_roll_neutral, &ir_pitch_neutral);
  if (!estimator_flight_time && (estimator_hspeed_mod > MIN_SPEED_FOR_TAKEOFF)) {
    estimator_flight_time = 1;
    launch = TRUE; /* Not set in non auto launch */
    DOWNLINK_SEND_TAKEOFF(&cputime);
  }
}
