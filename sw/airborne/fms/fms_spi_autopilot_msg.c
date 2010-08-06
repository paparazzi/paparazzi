/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "fms_debug.h"
#include "fms_spi_link.h"
#include "fms_autopilot_msg.h"
#include "fms_spi_autopilot_msg.h"

/* all these for telemetry */
#include "messages2.h"
#include "downlink_transport.h"
#include "udp_transport2.h"
#include "fms/fms_network.h"
#include "ready_main.h"

#include "airframe.h"
#include "actuators.h"
#include "rdyb_booz_imu.h"
#include "booz_radio_control.h"
#include "rdyb_mahrs.h"

static struct BoozImuFloat imu;
static struct FloatEulers body_to_imu_eulers = LISA_BODY_TO_IMU_EULERS;

static void (* vane_callback)(uint8_t vane_id, float alpha, float beta) = NULL;
static void (* pressure_callback)(uint8_t pressure_id, uint32_t pressure1, uint32_t pressure2) = NULL;
static void (* radio_control_callback)(void) = NULL;

void spi_ap_link_downlink_send(struct DownlinkTransport *tp)
{
  uint32_t timestamp = 0;
  DOWNLINK_SEND_EKF7_Y(tp, &timestamp, &imu.accel.x, &imu.accel.y, &imu.accel.z,
		    &imu.mag.x, &imu.mag.y, &imu.mag.z,
		    &imu.gyro.p, &imu.gyro.q, &imu.gyro.r);
}

void spi_ap_link_set_vane_callback(void (* vane_cb)(uint8_t vane_id, float alpha, float beta))
{
  vane_callback = vane_cb;
}

void spi_ap_link_set_pressure_callback(void (* pressure_cb)(uint8_t pressure_id, uint32_t pressure1, uint32_t pressure2))
{
  pressure_callback = pressure_cb;
}

void spi_ap_link_set_radio_control_callback(void (* radio_control_cb)(void))
{
  radio_control_callback = radio_control_cb;
}

int spi_ap_link_init()
{
  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return -1;
  }

  // Initialize IMU->Body orientation
  imu.body_to_imu_eulers = body_to_imu_eulers;

  FLOAT_QUAT_OF_EULERS(imu.body_to_imu_quat, imu.body_to_imu_eulers);
  FLOAT_QUAT_NORMALISE(imu.body_to_imu_quat);
  FLOAT_RMAT_OF_QUAT(imu.body_to_imu_rmat, imu.body_to_imu_quat);

  struct FloatRates bias0 = { 0., 0., 0.};
  rdyb_mahrs_init(imu.body_to_imu_quat, bias0);

  return 0;
}

static void passthrough_up_parse(struct AutopilotMessagePTUp *msg_up)
{

  if (msg_up->valid.vane && vane_callback)
    // FIXME: placeholders since the vane and pressure data fields don't exist yet
    vane_callback(0, 0., 0.);

  if (msg_up->valid.pressure && pressure_callback)
    pressure_callback(0, 0, 0);

  // Fill radio data
  if (msg_up->valid.rc && radio_control_callback) {
    radio_control.values[RADIO_CONTROL_ROLL] = msg_up->rc_roll;
    radio_control.values[RADIO_CONTROL_PITCH] = msg_up->rc_pitch;
    radio_control.values[RADIO_CONTROL_YAW] = msg_up->rc_yaw;
    radio_control.values[RADIO_CONTROL_THROTTLE] = msg_up->rc_thrust;
    radio_control.values[RADIO_CONTROL_MODE] = msg_up->rc_mode;
    radio_control.values[RADIO_CONTROL_KILL] = msg_up->rc_kill;
    radio_control.values[RADIO_CONTROL_GEAR] = msg_up->rc_gear;
    radio_control.values[RADIO_CONTROL_AUX3] = msg_up->rc_aux3;
    radio_control.values[RADIO_CONTROL_AUX4] = msg_up->rc_aux4;
    radio_control_callback();
  }
  // always fill status, it may change even when in the case when there is no new data
  radio_control.status = msg_up->rc_status;

  // Fill IMU data
  imu.gyro.p = RATE_FLOAT_OF_BFP(msg_up->gyro.p);
  imu.gyro.q = RATE_FLOAT_OF_BFP(msg_up->gyro.q);
  imu.gyro.r = RATE_FLOAT_OF_BFP(msg_up->gyro.r);

  imu.accel.x = ACCEL_FLOAT_OF_BFP(msg_up->accel.x);
  imu.accel.y = ACCEL_FLOAT_OF_BFP(msg_up->accel.y);
  imu.accel.z = ACCEL_FLOAT_OF_BFP(msg_up->accel.z);

  imu.mag.x = MAG_FLOAT_OF_BFP(msg_up->mag.x);
  imu.mag.y = MAG_FLOAT_OF_BFP(msg_up->mag.y);
  imu.mag.z = MAG_FLOAT_OF_BFP(msg_up->mag.z);

  if (msg_up->valid.imu)
    rdyb_booz_imu_update(&imu);
}

static void passthrough_down_fill(struct AutopilotMessagePTDown *msg_out)
{
  for (int i = 0; i < LISA_PWM_OUTPUT_NB; i++) {
    msg_out->pwm_outputs_usecs[i] = actuators[i];
  }
}

void spi_ap_link_periodic()
{
  static struct AutopilotMessagePTUp msg_in;
  static struct AutopilotMessagePTDown msg_out;

  passthrough_down_fill(&msg_out);

  // SPI transcieve
  spi_link_send(&msg_out, sizeof(union AutopilotMessage), &msg_in);

  passthrough_up_parse(&msg_in);
}
