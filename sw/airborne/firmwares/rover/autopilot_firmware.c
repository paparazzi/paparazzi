/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file firmwares/rover/autopilot.c
 *
 * Rover specific autopilot interface
 * and initialization
 */

#include "firmwares/rover/autopilot_firmware.h"

#include "generated/modules.h"

#include <stdint.h>
#include "modules/energy/electrical.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/radio_control/radio_control.h"

#if USE_GPS
#include "subsystems/gps.h"
#else
#if NO_GPS_NEEDED_FOR_NAV
#define GpsIsLost() FALSE
#else
#define GpsIsLost() TRUE
#endif
#endif


/* Geofence exceptions */
#include "modules/nav/nav_geofence.h"

#if USE_MOTOR_MIXING
#include "subsystems/actuators/motor_mixing.h"
#endif

static void send_status(struct transport_tx *trans, struct link_device *dev)
{
#if USE_GPS
  uint8_t fix = gps.fix;
#else
  uint8_t fix = 0;
#endif
  uint8_t motors_on = autopilot.motors_on;
  uint16_t time_sec = sys_time.nb_sec;
  pprz_msg_send_ROVER_STATUS(trans, dev, AC_ID,
      &radio_control.status, &radio_control.frame_rate,
      &fix, &autopilot.mode, &motors_on,
      &time_sec, &electrical.vsupply);
}

static void send_energy(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t throttle = 100 * autopilot.throttle / MAX_PPRZ;
  float power = electrical.vsupply * electrical.current;
  pprz_msg_send_ENERGY(trans, dev, AC_ID,
                       &throttle, &electrical.vsupply, &electrical.current, &power, &electrical.charge, &electrical.energy);
}

static void send_fp(struct transport_tx *trans, struct link_device *dev)
{
  int32_t carrot_east = POS_BFP_OF_REAL(nav.carrot.x);
  int32_t carrot_north = POS_BFP_OF_REAL(nav.carrot.y);
  int32_t carrot_up = 0;
  int32_t carrot_heading = ANGLE_BFP_OF_REAL(nav.heading);
  int32_t cmd = (int32_t)autopilot.throttle;
  pprz_msg_send_ROTORCRAFT_FP(trans, dev, AC_ID,
                              &(stateGetPositionEnu_i()->x),
                              &(stateGetPositionEnu_i()->y),
                              &(stateGetPositionEnu_i()->z),
                              &(stateGetSpeedEnu_i()->x),
                              &(stateGetSpeedEnu_i()->y),
                              &(stateGetSpeedEnu_i()->z),
                              &(stateGetNedToBodyEulers_i()->phi),
                              &(stateGetNedToBodyEulers_i()->theta),
                              &(stateGetNedToBodyEulers_i()->psi),
                              &carrot_east,
                              &carrot_north,
                              &carrot_up,
                              &carrot_heading,
                              &cmd,
                              &autopilot.flight_time);
}


#ifdef RADIO_CONTROL
static void send_rotorcraft_rc(struct transport_tx *trans, struct link_device *dev)
{
#ifdef RADIO_KILL_SWITCH
  int16_t _kill_switch = radio_control.values[RADIO_KILL_SWITCH];
#else
  int16_t _kill_switch = 42;
#endif
  pprz_msg_send_ROTORCRAFT_RADIO_CONTROL(trans, dev, AC_ID,
                                         &radio_control.values[RADIO_ROLL],
                                         &radio_control.values[RADIO_PITCH],
                                         &radio_control.values[RADIO_YAW],
                                         &radio_control.values[RADIO_THROTTLE],
                                         &radio_control.values[RADIO_MODE],
                                         &_kill_switch,
                                         &radio_control.status);
}
#endif

void autopilot_firmware_init(void)
{
  // register messages
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROVER_STATUS, send_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ENERGY, send_energy);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_FP, send_fp);
#ifdef RADIO_CONTROL
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_RADIO_CONTROL, send_rotorcraft_rc);
#endif
}

/** autopilot event function
 *
 */
void autopilot_event(void)
{
}

