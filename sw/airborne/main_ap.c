/*
 * $Id$
 *  
 * Copyright (C) 2003-2006  Antoine Drouin
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

/** \file main_ap.c
 *  \brief AP ( AutoPilot ) process
 *
 *   This process is reponsible for the collecting the different sensors data, fusing them to obtain
 * aircraft attitude and running the different control loops
 */

#include <math.h>

#include "main_ap.h"

#include "interrupt_hw.h"
#include "init_hw.h"
#include "adc.h"
#include "pid.h"
#include "gps.h"
#include "infrared.h"
#include "gyro.h"
#include "ap_downlink.h"
#include "nav.h"
#include "autopilot.h"
#include "estimator.h"
#include "rc_settings.h"
#include "cam.h"
#include "traffic_info.h"
#include "link_mcu.h"
#include "sys_time.h"
#include "flight_plan.h"
#include "datalink.h"
#include "wavecard.h"
#include "xbee.h"

#ifdef LED
#include "led.h"
#endif

#ifdef TELEMETER
#include "srf08.h"
#endif


#define LOW_BATTERY_DECIVOLT (LOW_BATTERY*10)


/** FIXME: should be in rc_settings but required by telemetry (ap_downlink.h)*/
uint8_t rc_settings_mode = RC_SETTINGS_MODE_NONE;


/** Define minimal speed for takeoff in m/s */
#define MIN_SPEED_FOR_TAKEOFF 5.


uint8_t fatal_error_nb = 0;
static const uint16_t version = 1;

uint8_t pprz_mode = PPRZ_MODE_AUTO2;
uint8_t vertical_mode = VERTICAL_MODE_MANUAL;
uint8_t lateral_mode = LATERAL_MODE_MANUAL;

uint8_t ir_estim_mode = IR_ESTIM_MODE_ON;

bool_t rc_event_1, rc_event_2;

uint8_t vsupply;

static uint8_t  mcu1_status, mcu1_ppm_cpt;

static bool_t kill_throttle = FALSE;

float slider_1_val, slider_2_val;

bool_t launch = FALSE;

float energy; /** Fuel consumption */

bool_t gps_lost = FALSE;


#define LIGHT_MODE_OFF 0
#define LIGHT_MODE_ON 1
#define LIGHT_MODE_FLASH 2

uint8_t light_mode = 0;


#define Min(x, y) (x < y ? x : y)
#define Max(x, y) (x > y ? x : y)


/** \brief Update paparazzi mode
 */
static inline uint8_t pprz_mode_update( void ) {
  /** We remain in home mode until explicit reset from the RC */
  if ((pprz_mode != PPRZ_MODE_HOME &&
       pprz_mode != PPRZ_MODE_GPS_OUT_OF_ORDER)
      || CheckEvent(rc_event_1)) {
    return ModeUpdate(pprz_mode, PPRZ_MODE_OF_PULSE(fbw_state->channels[RADIO_MODE], fbw_state->status));
  } else
    return FALSE;
}

#ifdef RADIO_LLS
/** \fn inline uint8_t ir_estim_mode_update( void )
 *  \brief update ir estimation if RADIO_LLS is true \n
 */
static inline uint8_t ir_estim_mode_update( void ) {
  return ModeUpdate(ir_estim_mode, IR_ESTIM_MODE_OF_PULSE(fbw_state->channels[RADIO_LLS]));
}
#endif


static inline uint8_t mcu1_status_update( void ) {
  uint8_t new_status = fbw_state->status;
  if (mcu1_status != new_status) {
    bool_t changed = ((mcu1_status&MASK_FBW_CHANGED) != (new_status&MASK_FBW_CHANGED));
    mcu1_status = new_status;
    return changed;
  }
  return FALSE;
}

/** Minimum delay before an event is enabled */
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
  }


#define Event(_cpt, _channel, _event, _cond) \
 EventUpdate(_cpt, (RcSettingsOff() && fbw_state->channels[_channel] _cond), _event)

#define EventPos(_cpt, _channel, _event) \
  Event(_cpt, _channel, _event, >(int)(0.75*MAX_PPRZ))

#define EventNeg(_cpt, _channel, _event) \
 Event(_cpt, _channel, _event, <(int)(-0.75*MAX_PPRZ))



static inline void events_update( void ) {
  static uint16_t event1_cpt = 0;
  EventPos(event1_cpt, RADIO_GAIN1, rc_event_1);
  static uint16_t event2_cpt = 0;
  EventNeg(event2_cpt, RADIO_GAIN1, rc_event_2);
}  


/** \brief Send back uncontrolled channels (actually only rudder)
 */
static inline void copy_from_to_fbw ( void ) {
#ifdef COMMAND_YAW /* FIXME */
  ap_state->commands[COMMAND_YAW] = fbw_state->channels[RADIO_YAW];
#endif
}



/* 
   called at 20Hz.
   sends a serie of initialisation messages followed by a stream of periodic ones
*/ 

/** Define number of message at initialisation */
#define INIT_MSG_NB 2

uint8_t ac_ident = AC_ID;

/** \brief Send a serie of initialisation messages followed by a stream of periodic ones
 *
 * Called at 10Hz.
 */
static inline void reporting_task( void ) {
  static uint8_t boot = TRUE;

  /** initialisation phase during boot */
  if (boot) {
    DOWNLINK_SEND_BOOT(&version);
    boot = FALSE;
  }
  /** then report periodicly */
  else {
    PeriodicSendAp();
  }
}

/** \brief Function to be called when a command is available (usually comming from the radio command)
 */
inline void telecommand_task( void ) {
  uint8_t mode_changed = FALSE;
  copy_from_to_fbw();
  
  uint8_t really_lost = bit_is_set(fbw_state->status, RADIO_REALLY_LOST) && (pprz_mode == PPRZ_MODE_AUTO1 || pprz_mode == PPRZ_MODE_MANUAL);
  if (launch && (really_lost || too_far_from_home)) {
    pprz_mode = PPRZ_MODE_HOME;
    mode_changed = TRUE;
  }
  if (bit_is_set(fbw_state->status, AVERAGED_CHANNELS_SENT)) {
    bool_t pprz_mode_changed = pprz_mode_update();
    mode_changed |= pprz_mode_changed;
#ifdef RADIO_LLS
    mode_changed |= ir_estim_mode_update();
#endif
#if defined RADIO_CALIB && defined RADIO_CONTROL_SETTINGS
    bool_t calib_mode_changed = RcSettingsModeUpdate(fbw_state->channels);
    rc_settings(calib_mode_changed || pprz_mode_changed);
    mode_changed |= calib_mode_changed;
#endif
  }
  mode_changed |= mcu1_status_update();
  if ( mode_changed )
    PERIODIC_SEND_PPRZ_MODE();
  
  /** If Auto1 mode, compute \a desired_roll and \a desired_pitch from 
   * \a RADIO_ROLL and \a RADIO_PITCH \n
   * Else asynchronously set by \a course_pid_run
   */
  if (pprz_mode == PPRZ_MODE_AUTO1) {
    /** In Auto1 mode, roll is bounded between [-AUTO1_MAX_ROLL;AUTO1_MAX_ROLL] */
    desired_roll = FLOAT_OF_PPRZ(fbw_state->channels[RADIO_ROLL], 0., -AUTO1_MAX_ROLL);
    
    /** In Auto1 mode, pitch is bounded between [-AUTO1_MAX_PITCH;AUTO1_MAX_PITCH] */
    desired_pitch = FLOAT_OF_PPRZ(fbw_state->channels[RADIO_PITCH], 0., AUTO1_MAX_PITCH);
  }
  if (pprz_mode == PPRZ_MODE_MANUAL || pprz_mode == PPRZ_MODE_AUTO1) {
    desired_gaz = fbw_state->channels[RADIO_THROTTLE];
  }
  /** else asynchronously set by climb_pid_run(); */
  
  mcu1_ppm_cpt = fbw_state->ppm_cpt;
  vsupply = fbw_state->vsupply;
  
  events_update();
  
  if (!estimator_flight_time) {
#ifdef INFRARED
    ground_calibrate(STICK_PUSHED(fbw_state->channels[RADIO_ROLL]));
#endif
    if (pprz_mode == PPRZ_MODE_AUTO2 && fbw_state->channels[RADIO_THROTTLE] > GAZ_THRESHOLD_TAKEOFF) {
      launch = TRUE;
    }
  }
}

/** \fn void navigation_task( void )
 *  \brief Compute desired_course
 */
static void navigation_task( void ) {
#if defined FAILSAFE_DELAY_WITHOUT_GPS
  /** This section is used for the failsafe of GPS */
  static uint8_t last_pprz_mode;
  /** Test if we lost the GPS */
  if (!GPS_FIX_VALID(gps_mode) ||
      (cpu_time - last_gps_msg_t > FAILSAFE_DELAY_WITHOUT_GPS)) {
    /** If aircraft is launch and is in autonomus mode, go into
	PPRZ_MODE_GPS_OUT_OF_ORDER mode (Failsafe). */
    if (launch && (pprz_mode == PPRZ_MODE_AUTO2 ||
		   pprz_mode == PPRZ_MODE_HOME)) {
      last_pprz_mode = pprz_mode;
      pprz_mode = PPRZ_MODE_GPS_OUT_OF_ORDER;
      PERIODIC_SEND_PPRZ_MODE();
      gps_lost = TRUE;
    }
  }
  /** If aircraft was in failsafe mode, come back in previous mode */
  else if (gps_lost) {
    pprz_mode = last_pprz_mode;
    gps_lost = FALSE;
    PERIODIC_SEND_PPRZ_MODE();
  }
#endif /* GPS && FAILSAFE_DELAY_WITHOUT_GPS */
  
  /** Default to keep compatibility with previous behaviour */
  lateral_mode = LATERAL_MODE_COURSE;
  if (pprz_mode == PPRZ_MODE_HOME)
    nav_home();
  else if (pprz_mode == PPRZ_MODE_GPS_OUT_OF_ORDER)
    nav_without_gps();
  else
    nav_update();
  
  SEND_NAVIGATION();

  SEND_CAM();
  
  
  if (pprz_mode == PPRZ_MODE_AUTO2 || pprz_mode == PPRZ_MODE_HOME
			|| pprz_mode == PPRZ_MODE_GPS_OUT_OF_ORDER) {
    if (lateral_mode >=LATERAL_MODE_COURSE)
      course_pid_run(); /* aka compute nav_desired_roll */
    desired_roll = nav_desired_roll;
    if (vertical_mode == VERTICAL_MODE_AUTO_ALT)
      altitude_pid_run();
    if (vertical_mode >= VERTICAL_MODE_AUTO_CLIMB)
      climb_pid_run();
    if (vertical_mode == VERTICAL_MODE_AUTO_GAZ)
      desired_gaz = nav_desired_gaz;
    desired_pitch = nav_pitch;
    if (kill_throttle || (!estimator_flight_time && !launch))
      desired_gaz = 0;
  }  
  energy += (float)desired_gaz * (MILLIAMP_PER_PERCENT / MAX_PPRZ * 0.25);
}


#ifndef KILL_MODE_DISTANCE
#define KILL_MODE_DISTANCE (1.5*MAX_DIST_FROM_HOME)
#endif 


/** Maximum time allowed for low battery level */
#define LOW_BATTERY_DELAY 5

/** \fn inline void periodic_task( void )
 *  \brief Do periodic tasks at 61 Hz
 */
/**There are four @@@@@ boucles @@@@@:
 * - 20 Hz:
 *   - lets use \a reporting_task at 10 Hz
 *   - updates ir with \a ir_update
 *   - updates estimator of ir with \a estimator_update_state_infrared
 *   - set \a desired_aileron and \a desired_elevator with \a roll_pitch_pid_run
 *   - sends to \a fbw \a desired_gaz, \a desired_aileron and
 *     \a desired_elevator \note \a desired_gaz is set upon GPS
 *     message reception
 * - 10 Hz: to get a \a stage_time_ds
 * - 4 Hz:
 *   - calls \a estimator_propagate_state
 *   - do navigation with \a navigation_task
 *
 */

void periodic_task_ap( void ) {
  static uint8_t _20Hz   = 0;
  static uint8_t _10Hz   = 0;
  static uint8_t _4Hz   = 0;
  static uint8_t _1Hz   = 0;

  //  estimator_t += PERIOD;

  _20Hz++;
  if (_20Hz>=3) _20Hz=0;
  _10Hz++;
  if (_10Hz>=6) _10Hz=0;
  _4Hz++;
  if (_4Hz>=15) _4Hz=0;
  _1Hz++;
  if (_1Hz>=61) _1Hz=0;
	  
 
  
  if (!_10Hz) {
    stage_time_ds = stage_time_ds + .1;
    reporting_task();
  }

  if (!_1Hz) {
    if (estimator_flight_time) estimator_flight_time++;
    cpu_time++;
    stage_time_ds = (int16_t)(stage_time_ds+.5);
    stage_time++;
    block_time++;

    static uint8_t t = 0;
    if (vsupply < LOW_BATTERY_DECIVOLT) t++; else t = 0;
    kill_throttle |= (t >= LOW_BATTERY_DELAY);
    kill_throttle |= launch && (dist2_to_home > Square(KILL_MODE_DISTANCE));
  }
  switch (_1Hz) {
#ifdef TELEMETER
  case 1:
    srf08_initiate_ranging();
    break;
  case 5:
    /** 65ms since initiate_ranging() (the spec ask for 65ms) */
    srf08_receive();
    break;
#endif

  }

  switch(_4Hz) {
  case 0:
    estimator_propagate_state();
    navigation_task();
    break;
  case 1:
    if (!estimator_flight_time && 
	estimator_hspeed_mod > MIN_SPEED_FOR_TAKEOFF) {
      estimator_flight_time = 1;
      launch = TRUE; /* Not set in non auto launch */
      DOWNLINK_SEND_TAKEOFF(&cpu_time);
    }
    break;

#ifdef LIGHT_PIN_1
  case 2:
    if (light_mode == LIGHT_MODE_OFF)
      LED_ON(LIGHT_PIN_1);
    else if (light_mode == LIGHT_MODE_ON)
      LED_OFF(LIGHT_PIN_1);
    else
      LED_TOGGLE(LIGHT_PIN_1);
#endif /* LIGHT_PIN_1 */

    /*  default: */
  }

#ifndef CONTROL_RATE
#define CONTROL_RATE 20
#endif

#if CONTROL_RATE != 60 && CONTROL_RATE != 20
#error "Only 20 and 60 allowed for CONTROL_RATE"
#endif

#if CONTROL_RATE == 20
  if (!_20Hz)
#endif
    {
#if defined GYRO
      gyro_update();
#endif
#ifdef INFRARED
      ir_update();
      estimator_update_state_infrared();
#endif /* INFRARED */
      roll_pitch_pid_run(); /* Set  desired_aileron & desired_elevator */
      pid_slew_gaz();
      ap_state->commands[COMMAND_THROTTLE] = desired_gaz; /* desired_gaz is set upon GPS message reception */
      ap_state->commands[COMMAND_ROLL] = desired_aileron;
      ap_state->commands[COMMAND_PITCH] = desired_elevator;
      
#if defined MCU_SPI_LINK
      link_mcu_send();
#elif defined INTER_MCU && defined SINGLE_MCU
      /**Directly set the flag indicating to FBW that shared buffer is available*/
      inter_mcu_received_ap = TRUE;
#endif
    }

}




#ifdef MCU_SPI_LINK /** ap alone, using SPI to communicate with fbw */
#include "spi.h"
#endif

#ifdef TELEMETER
#include "srf08.h"
#endif

void init_ap( void ) {
#ifdef LED
  led_init();
#endif
#ifndef SINGLE_MCU /** Dual mcus : init done in main_fbw */
  hw_init();
  sys_time_init(); 
#ifdef ADC
  adc_init();
#endif
#endif /* SINGLE_MCU */

  /************* Sensors initialization ***************/
#ifdef INFRARED
  ir_init();
#endif
#ifdef GYRO
  gyro_init();
#endif
#ifdef GPS
  gps_init();
#endif
#ifdef TELEMETER
  srf08_init();
#endif
#ifdef USE_UART0
  Uart0Init();
#endif
#ifdef USE_UART1
  Uart1Init();
#endif


  /************* Links initialization ***************/
#if defined MCU_SPI_LINK
  spi_init();
  link_mcu_init();
#endif
#ifdef MODEM
  modem_init();
#endif
#if defined DATALINK && DATALINK == WAVECARD
  /** Reset the wavecard during the init pause */
  wc_reset();
#endif
#if defined GPS && defined GPS_CONFIGURE
  gps_configure();
#endif

  /************ Internal status ***************/
  estimator_init();
  nav_init();


  /** - start interrupt task */
  int_enable();

  /** - wait 0.5s (for modem init ?) */
  uint8_t init_cpt = 30;
  while (init_cpt) {
    if (sys_time_periodic())
      init_cpt--;
  }

#if defined DATALINK

#if DATALINK == XBEE
  xbee_init();
#elif DATALINK == WAVECARD
  wc_end_reset();
  init_cpt = 60;
  while (init_cpt) {
    if (sys_time_periodic())
      init_cpt--;
  }
  wc_configure();
#endif
#endif /* DATALINK */

#if defined AEROCOMM_DATA_PIN
  IO0DIR |= _BV(AEROCOMM_DATA_PIN);
  IO0SET = _BV(AEROCOMM_DATA_PIN);
#endif
}


/*********** EVENT ***********************************************************/
void event_task_ap( void ) {
#ifdef GPS
#ifndef HITL /** else comes through the datalink */
  if (GpsBuffer()) {
    ReadGpsBuffer();
  }
#endif
  if (gps_msg_received) {
    /* parse and use GPS messages */
    parse_gps_msg();
    gps_msg_received = FALSE;
    if (gps_pos_available) {
      use_gps_pos();
      gps_pos_available = FALSE;
    }
  }
#endif /** GPS */

#if defined DATALINK 

#if DATALINK == PPRZ
  if (PprzBuffer()) {
    ReadPprzBuffer();
    if (pprz_msg_received) {
      pprz_parse_payload();
      pprz_msg_received = FALSE;
    }
  }
#elif DATALINK == XBEE
  if (XBeeBuffer()) {
    ReadXBeeBuffer();
    if (xbee_msg_received) {
      xbee_parse_payload();
      xbee_msg_received = FALSE;
    }
  }
#elif DATALINK == WAVECARD
  if (WavecardBuffer()) {
    ReadWavecardBuffer();
    if (wc_msg_received) {
      wc_parse_payload();
      wc_msg_received = FALSE;
    }
  }
#elif
#error "Unknown DATALINK"
#endif

  if (dl_msg_available) {
    dl_parse_msg();
    dl_msg_available = FALSE;
  }
#endif /** DATALINK */

#ifdef TELEMETER
  /** Handling of data sent by the device (initiated by srf08_receive() */
  if (srf08_received) {
    srf08_received = FALSE;
    srf08_read();
  }
  if (srf08_got) {
    srf08_got = FALSE;
    srf08_copy();
    DOWNLINK_SEND_RANGEFINDER(&srf08_range);
  }
#endif

#ifdef MCU_SPI_LINK
  if (spi_message_received) {
    /* Got a message on SPI. */
    spi_message_received = FALSE;
    link_mcu_event_task();
  }
#endif

  if (inter_mcu_received_fbw) {
    /* receive radio control task from fbw */
    inter_mcu_received_fbw = FALSE;
    telecommand_task();
  }
} 
