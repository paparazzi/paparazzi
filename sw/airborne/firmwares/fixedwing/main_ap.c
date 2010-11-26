/*
 * $Id$
 *
 * Copyright (C) 2003-2010  The Paparazzi Team
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

#define MODULES_C

#include <math.h>

#include "firmwares/fixedwing/main_ap.h"

#include "interrupt_hw.h"
#include "init_hw.h"
#include "adc.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/guidance/guidance_v.h"
#include "gps.h"
#include "infrared.h"
#include "gyro.h"
#include "ap_downlink.h"
#include "nav.h"
#include "autopilot.h"
#include "estimator.h"
#include "generated/settings.h"
#include "link_mcu.h"
#include "sys_time.h"
#include "generated/flight_plan.h"
#include "datalink.h"
#include "xbee.h"

#include "gpio.h"

#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
#include "rc_settings.h"
#endif

#ifdef LED
#include "led.h"
#endif

#if defined USE_I2C0 || USE_I2C1
#include "i2c.h"
#endif

#ifdef USE_SPI
#include "spi.h"
#endif

#ifdef TRAFFIC_INFO
#include "traffic_info.h"
#endif

#ifdef USE_USB_SERIAL
#include "usb_serial.h"
#endif

#if ! defined CATASTROPHIC_BAT_LEVEL && defined LOW_BATTERY
#warning "LOW_BATTERY deprecated. Renamed into CATASTROPHIC_BAT_LEVEL (in airframe file)"
#define CATASTROPHIC_BAT_LEVEL LOW_BATTERY
#endif

#define LOW_BATTERY_DECIVOLT (CATASTROPHIC_BAT_LEVEL*10)

#include "generated/modules.h"

/** FIXME: should be in rc_settings but required by telemetry (ap_downlink.h)*/
uint8_t rc_settings_mode = 0;

/** Define minimal speed for takeoff in m/s */
#define MIN_SPEED_FOR_TAKEOFF 5.

bool_t power_switch;
uint8_t fatal_error_nb = 0;
static const uint16_t version = 1;

uint8_t pprz_mode = PPRZ_MODE_AUTO2;
uint8_t lateral_mode = LATERAL_MODE_MANUAL;

static uint8_t  mcu1_status;

#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
static uint8_t  mcu1_ppm_cpt;
#endif

bool_t kill_throttle = FALSE;

float slider_1_val, slider_2_val;

bool_t launch = FALSE;

uint8_t vsupply;	// deciVolt
static int32_t current;	// milliAmpere

float energy;       // Fuel consumption (mAh)

bool_t gps_lost = FALSE;


#define Min(x, y) (x < y ? x : y)
#define Max(x, y) (x > y ? x : y)

/** \brief Update paparazzi mode
 */
#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
static inline uint8_t pprz_mode_update( void ) {
  if ((pprz_mode != PPRZ_MODE_HOME &&
       pprz_mode != PPRZ_MODE_GPS_OUT_OF_ORDER)
#ifdef UNLOCKED_HOME_MODE
      || TRUE
#endif
      ) {
    return ModeUpdate(pprz_mode, PPRZ_MODE_OF_PULSE(fbw_state->channels[RADIO_MODE], fbw_state->status));
  } else
    return FALSE;
}
#else // not RADIO_CONTROL
static inline uint8_t pprz_mode_update( void ) {
  return FALSE;
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


/** \brief Send back uncontrolled channels
 */
static inline void copy_from_to_fbw ( void ) {
#ifdef SetAutoCommandsFromRC
  SetAutoCommandsFromRC(ap_state->commands, fbw_state->channels);
#elif defined RADIO_YAW && defined COMMAND_YAW
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
 * Called at 60Hz.
 */
static inline void reporting_task( void ) {
  static uint8_t boot = TRUE;

  /** initialisation phase during boot */
  if (boot) {
    DOWNLINK_SEND_BOOT(DefaultChannel, &version);
    boot = FALSE;
  }
  /** then report periodicly */
  else {
    PeriodicSendAp(DefaultChannel);
  }
}

#ifndef RC_LOST_MODE
#define RC_LOST_MODE PPRZ_MODE_HOME
#endif

/** \brief Function to be called when a message from FBW is available */
static inline void telecommand_task( void ) {
  uint8_t mode_changed = FALSE;
  copy_from_to_fbw();

  uint8_t really_lost = bit_is_set(fbw_state->status, STATUS_RADIO_REALLY_LOST) && (pprz_mode == PPRZ_MODE_AUTO1 || pprz_mode == PPRZ_MODE_MANUAL);
  if (pprz_mode != PPRZ_MODE_HOME && pprz_mode != PPRZ_MODE_GPS_OUT_OF_ORDER && launch) {
    if  (too_far_from_home) {
      pprz_mode = PPRZ_MODE_HOME;
      mode_changed = TRUE;
    }
    if  (really_lost) {
      pprz_mode = RC_LOST_MODE;
      mode_changed = TRUE;
    }
  }
  if (bit_is_set(fbw_state->status, AVERAGED_CHANNELS_SENT)) {
    bool_t pprz_mode_changed = pprz_mode_update();
    mode_changed |= pprz_mode_changed;
#if defined RADIO_CALIB && defined RADIO_CONTROL_SETTINGS
    bool_t calib_mode_changed = RcSettingsModeUpdate(fbw_state->channels);
    rc_settings(calib_mode_changed || pprz_mode_changed);
    mode_changed |= calib_mode_changed;
#endif
  }
  mode_changed |= mcu1_status_update();
  if ( mode_changed )
    PERIODIC_SEND_PPRZ_MODE(DefaultChannel);

#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
  /** In AUTO1 mode, compute roll setpoint and pitch setpoint from
   * \a RADIO_ROLL and \a RADIO_PITCH \n
   */
  if (pprz_mode == PPRZ_MODE_AUTO1) {
    /** Roll is bounded between [-AUTO1_MAX_ROLL;AUTO1_MAX_ROLL] */
    h_ctl_roll_setpoint = FLOAT_OF_PPRZ(fbw_state->channels[RADIO_ROLL], 0., -AUTO1_MAX_ROLL);

    /** Pitch is bounded between [-AUTO1_MAX_PITCH;AUTO1_MAX_PITCH] */
    h_ctl_pitch_setpoint = FLOAT_OF_PPRZ(fbw_state->channels[RADIO_PITCH], 0., AUTO1_MAX_PITCH);
  } /** Else asynchronously set by \a h_ctl_course_loop() */

  /** In AUTO1, throttle comes from RADIO_THROTTLE
      In MANUAL, the value is copied to get it in the telemetry */
  if (pprz_mode == PPRZ_MODE_MANUAL || pprz_mode == PPRZ_MODE_AUTO1) {
    v_ctl_throttle_setpoint = fbw_state->channels[RADIO_THROTTLE];
  }
  /** else asynchronously set by v_ctl_climb_loop(); */

  mcu1_ppm_cpt = fbw_state->ppm_cpt;
#endif // RADIO_CONTROL


  vsupply = fbw_state->vsupply;
  current = fbw_state->current;

#ifdef RADIO_CONTROL
  if (!estimator_flight_time) {
    if (pprz_mode == PPRZ_MODE_AUTO2 && fbw_state->channels[RADIO_THROTTLE] > THROTTLE_THRESHOLD_TAKEOFF) {
      launch = TRUE;
    }
  }
#endif
}

/** \fn void navigation_task( void )
 *  \brief Compute desired_course
 */
static void navigation_task( void ) {
#if defined FAILSAFE_DELAY_WITHOUT_GPS
  /** This section is used for the failsafe of GPS */
  static uint8_t last_pprz_mode;

  /** If aircraft is launched and is in autonomus mode, go into
      PPRZ_MODE_GPS_OUT_OF_ORDER mode (Failsafe) if we lost the GPS */
  if (launch) {
    if (GpsTimeoutError) {
      if (pprz_mode == PPRZ_MODE_AUTO2 || pprz_mode == PPRZ_MODE_HOME) {
    last_pprz_mode = pprz_mode;
    pprz_mode = PPRZ_MODE_GPS_OUT_OF_ORDER;
    PERIODIC_SEND_PPRZ_MODE(DefaultChannel);
    gps_lost = TRUE;
      }
    } else /* GPS is ok */ if (gps_lost) {
      /** If aircraft was in failsafe mode, come back in previous mode */
      pprz_mode = last_pprz_mode;
      gps_lost = FALSE;

      PERIODIC_SEND_PPRZ_MODE(DefaultChannel);
    }
  }
#endif /* GPS && FAILSAFE_DELAY_WITHOUT_GPS */

  common_nav_periodic_task_4Hz();
  if (pprz_mode == PPRZ_MODE_HOME)
    nav_home();
  else if (pprz_mode == PPRZ_MODE_GPS_OUT_OF_ORDER)
    nav_without_gps();
  else
    nav_periodic_task();

#ifdef TCAS
  CallTCAS();
#endif

#ifndef PERIOD_NAVIGATION_DefaultChannel_0 // If not sent periodically (in default 0 mode)
  SEND_NAVIGATION(DefaultChannel);
#endif

  SEND_CAM(DefaultChannel);

  /* The nav task computes only nav_altitude. However, we are interested
     by desired_altitude (= nav_alt+alt_shift) in any case.
     So we always run the altitude control loop */
  if (v_ctl_mode == V_CTL_MODE_AUTO_ALT)
    v_ctl_altitude_loop();

  if (pprz_mode == PPRZ_MODE_AUTO2 || pprz_mode == PPRZ_MODE_HOME
            || pprz_mode == PPRZ_MODE_GPS_OUT_OF_ORDER) {
#ifdef H_CTL_RATE_LOOP
    /* Be sure to be in attitude mode, not roll */
    h_ctl_auto1_rate = FALSE;
#endif
    if (lateral_mode >=LATERAL_MODE_COURSE)
      h_ctl_course_loop(); /* aka compute nav_desired_roll */
    if (v_ctl_mode >= V_CTL_MODE_AUTO_CLIMB)
      v_ctl_climb_loop();
    if (v_ctl_mode == V_CTL_MODE_AUTO_THROTTLE)
      v_ctl_throttle_setpoint = nav_throttle_setpoint;

#if defined V_CTL_THROTTLE_IDLE
    Bound(v_ctl_throttle_setpoint, TRIM_PPRZ(V_CTL_THROTTLE_IDLE*MAX_PPRZ), MAX_PPRZ);
#endif

#ifdef V_CTL_POWER_CTL_BAT_NOMINAL
    if (vsupply > 0.) {
      v_ctl_throttle_setpoint *= 10. * V_CTL_POWER_CTL_BAT_NOMINAL / (float)vsupply;
      v_ctl_throttle_setpoint = TRIM_UPPRZ(v_ctl_throttle_setpoint);
    }
#endif

    h_ctl_pitch_setpoint = nav_pitch;
    Bound(h_ctl_pitch_setpoint, H_CTL_PITCH_MIN_SETPOINT, H_CTL_PITCH_MAX_SETPOINT);
    if (kill_throttle || (!estimator_flight_time && !launch))
      v_ctl_throttle_setpoint = 0;
  }
  energy += ((float)current) / 3600.0f * 0.25f;	// mAh = mA * dt (4Hz -> hours)
}


#ifndef KILL_MODE_DISTANCE
#define KILL_MODE_DISTANCE (1.5*MAX_DIST_FROM_HOME)
#endif


/** Maximum time allowed for low battery level */
#define LOW_BATTERY_DELAY 5

/** \fn inline void periodic_task( void )
 *  \brief Do periodic tasks at 60 Hz
 */
/**There are four @@@@@ boucles @@@@@:
 * - 20 Hz:
 *   - lets use \a reporting_task at 60 Hz
 *   - updates ir with \a ir_update
 *   - updates estimator of ir with \a estimator_update_state_infrared
 *   - set \a desired_aileron and \a desired_elevator with \a pid_attitude_loop
 *   - sends to \a fbw \a desired_throttle, \a desired_aileron and
 *     \a desired_elevator \note \a desired_throttle is set upon GPS
 *     message reception
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

  _20Hz++;
  if (_20Hz>=3) _20Hz=0;
  _10Hz++;
  if (_10Hz>=6) _10Hz=0;
  _4Hz++;
  if (_4Hz>=15) _4Hz=0;
  _1Hz++;
  if (_1Hz>=60) _1Hz=0;

  reporting_task();

  if (!_1Hz) {
    if (estimator_flight_time) estimator_flight_time++;
#if defined DATALINK || defined SITL
    datalink_time++;
#endif

    static uint8_t t = 0;
    if (vsupply < LOW_BATTERY_DECIVOLT) t++; else t = 0;
    kill_throttle |= (t >= LOW_BATTERY_DELAY);
    kill_throttle |= launch && (dist2_to_home > Square(KILL_MODE_DISTANCE));
  }

  switch(_4Hz) {
  case 0:
#ifdef SITL
#ifdef GPS_TRIGGERED_FUNCTION
    GPS_TRIGGERED_FUNCTION();
#endif
#endif
    estimator_propagate_state();
#ifdef EXTRA_DOWNLINK_DEVICE
    DOWNLINK_SEND_ATTITUDE(ExtraPprzTransport,&estimator_phi,&estimator_psi,&estimator_theta);
#endif
    navigation_task();
    break;
  case 1:
    if (!estimator_flight_time &&
    estimator_hspeed_mod > MIN_SPEED_FOR_TAKEOFF) {
      estimator_flight_time = 1;
      launch = TRUE; /* Not set in non auto launch */
      DOWNLINK_SEND_TAKEOFF(DefaultChannel, &cpu_time_sec);
  default:
    break;
    }

    break;

#ifdef USE_GPIO
  case 3:
    GpioUpdate1();
    break;
#endif

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

#ifdef USE_GYRO
      gyro_update();
#endif

#ifdef USE_INFRARED
      ir_update();
      estimator_update_state_infrared();
#endif /* USE_INFRARED */
      h_ctl_attitude_loop(); /* Set  h_ctl_aileron_setpoint & h_ctl_elevator_setpoint */
      v_ctl_throttle_slew();
      ap_state->commands[COMMAND_THROTTLE] = v_ctl_throttle_slewed;
      ap_state->commands[COMMAND_ROLL] = h_ctl_aileron_setpoint;
      ap_state->commands[COMMAND_PITCH] = h_ctl_elevator_setpoint;

#if defined MCU_SPI_LINK
      link_mcu_send();
#elif defined INTER_MCU && defined SINGLE_MCU
      /**Directly set the flag indicating to FBW that shared buffer is available*/
      inter_mcu_received_ap = TRUE;
#endif

    }

  modules_periodic_task();
}


void init_ap( void ) {
#ifndef SINGLE_MCU /** init done in main_fbw in single MCU */
  hw_init();
  sys_time_init();

#ifdef LED
  led_init();
#endif

#ifdef ADC
  adc_init();
#endif
#endif /* SINGLE_MCU */

  /************* Sensors initialization ***************/
#ifdef USE_INFRARED
  ir_init();
#endif
#ifdef USE_GYRO
  gyro_init();
#endif
#ifdef USE_GPS
  gps_init();
#endif
#ifdef USE_UART0
  Uart0Init();
#endif
#ifdef USE_UART1
  Uart1Init();
#endif
#ifdef USE_UART2
  Uart2Init();
#endif
#ifdef USE_UART3
  Uart3Init();
#endif
#ifdef USE_USB_SERIAL
  VCOM_init();
#endif

#ifdef USE_GPIO
  GpioInit();
#endif

#ifdef USE_I2C0
  i2c0_init();
#endif
#ifdef USE_I2C1
  i2c1_init();
#endif
#ifdef USE_I2C2
  i2c2_init();
#endif

  /************* Links initialization ***************/
#if defined USE_SPI
  spi_init();
#endif
#if defined MCU_SPI_LINK
  link_mcu_init();
#endif
#ifdef MODEM
  modem_init();
#endif

  /************ Internal status ***************/
  h_ctl_init();
  v_ctl_init();
  estimator_init();
#ifdef ALT_KALMAN
  alt_kalman_init();
#endif
  nav_init();

  modules_init();

  /** - start interrupt task */
  int_enable();

  /** wait 0.5s (historical :-) */
  sys_time_usleep(500000);

#if defined GPS_CONFIGURE
  gps_configure_uart();
#endif

#if defined DATALINK

#if DATALINK == XBEE
  xbee_init();
#endif
#endif /* DATALINK */

#if defined AEROCOMM_DATA_PIN
  IO0DIR |= _BV(AEROCOMM_DATA_PIN);
  IO0SET = _BV(AEROCOMM_DATA_PIN);
#endif

  power_switch = FALSE;

  /************ Multi-uavs status ***************/

#ifdef TRAFFIC_INFO
  traffic_info_init();
#endif

}


/*********** EVENT ***********************************************************/
void event_task_ap( void ) {

#ifdef USE_GPS
#if !(defined HITL) && !(defined UBX_EXTERNAL) /** else comes through the datalink */
  if (GpsBuffer()) {
    ReadGpsBuffer();
  }
#endif
  if (gps_msg_received) {
    /* parse and use GPS messages */
#ifdef GPS_CONFIGURE
    if (gps_configuring)
      gps_configure();
    else
#endif
      parse_gps_msg();
    gps_msg_received = FALSE;
    if (gps_pos_available) {
      gps_verbose_downlink = !launch;
      UseGpsPosNoSend(estimator_update_state_gps);
      gps_downlink();
#ifdef GPS_TRIGGERED_FUNCTION
#ifndef SITL
    GPS_TRIGGERED_FUNCTION();
#endif
#endif
      gps_pos_available = FALSE;
    }
  }
#endif /** USE_GPS */


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
#else
#error "Unknown DATALINK"
#endif

  if (dl_msg_available) {
    dl_parse_msg();
    dl_msg_available = FALSE;
  }
#endif /** DATALINK */

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

  modules_event_task();
} /* event_task_ap() */
