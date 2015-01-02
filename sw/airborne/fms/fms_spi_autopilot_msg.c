/*
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

#include "generated/airframe.h"
/* sort of a hack, we're not really fixed wing here but we need their declarations */
#include "firmwares/fixedwing/actuators.h"
#include "rdyb_booz_imu.h"
#include "subsystems/radio_control.h"
#include "rdyb_mahrs.h"

static struct ImuFloat imuFloat;
static struct FloatQuat body_to_imu_quat = IMU_POSE_BODY_TO_IMU_QUAT;

static void (* vane_callback)(uint8_t vane_id, float alpha, float beta) = NULL;
static void (* pressure_absolute_callback)(uint8_t pressure_id, uint32_t pressure) = NULL;
static void (* pressure_differential_callback)(uint8_t pressure_id, uint32_t pressure) = NULL;
static void (* radio_control_callback)(void) = NULL;
static void (* adc_callback)(uint16_t *adc_channels) = NULL;

void spi_ap_link_downlink_send(struct DownlinkTransport *tp)
{
  uint32_t timestamp = 0;
  DOWNLINK_SEND_EKF7_Y(tp, &timestamp, &imuFloat.accel.x, &imuFloat.accel.y, &imuFloat.accel.z,
                       &imuFloat.mag.x, &imuFloat.mag.y, &imuFloat.mag.z,
                       &imuFloat.gyro.p, &imuFloat.gyro.q, &imuFloat.gyro.r);
}

void spi_ap_link_set_vane_callback(void (* vane_cb)(uint8_t vane_id, float alpha, float beta))
{
  vane_callback = vane_cb;
}

void spi_ap_link_set_pressure_absolute_callback(void (* pressure_absolute_cb)(uint8_t pressure_id, uint32_t pressure))
{
  pressure_absolute_callback = pressure_absolute_cb;
}

void spi_ap_link_set_pressure_differential_callback(void (* pressure_differential_cb)(uint8_t pressure_id,
    uint32_t pressure))
{
  pressure_differential_callback = pressure_differential_cb;
}

void spi_ap_link_set_radio_control_callback(void (* radio_control_cb)(void))
{
  radio_control_callback = radio_control_cb;
}

void spi_ap_link_set_adc_callback(void (* adc_callback_fun)(uint16_t *adc_channels))
{
  adc_callback = adc_callback_fun;
}

int spi_ap_link_init()
{
  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return -1;
  }

  // Initialize IMU->Body orientation
  imuFloat.body_to_imu_quat = body_to_imu_quat;
  imuFloat.sample_count = 0;

#ifdef IMU_ALIGN_BENCH
  // This code is for aligning body to imu rotation, turn this on, put the vehicle in hover, pointed north, read BOOZ2_AHRS_REF_QUAT as body to imu (in wing frame)
  struct FloatVect3 x_axis = { 0.0, 1.0, 0.0 };
  FLOAT_QUAT_OF_AXIS_ANGLE(imuFloat.body_to_imu_quat, x_axis, QUAT_SETPOINT_HOVER_PITCH);
#endif

  FLOAT_QUAT_NORMALIZE(imuFloat.body_to_imu_quat);
  FLOAT_EULERS_OF_QUAT(imuFloat.body_to_imu_eulers, imuFloat.body_to_imu_quat);
  FLOAT_RMAT_OF_QUAT(imuFloat.body_to_imu_rmat, imuFloat.body_to_imu_quat);

  struct FloatRates bias0 = { 0., 0., 0.};
  rdyb_mahrs_init(imuFloat.body_to_imu_quat, bias0);

  return 0;
}

static void passthrough_up_parse(struct AutopilotMessagePTUp *msg_up)
{

  if (msg_up->valid.vane && vane_callback) {
    vane_callback(0, msg_up->vane_angle1, msg_up->vane_angle2);
  }

  // Fill pressure data
  if (msg_up->valid.pressure_absolute && pressure_absolute_callback) {
    pressure_absolute_callback(0, msg_up->pressure_absolute);
  }

  if (msg_up->valid.pressure_differential && pressure_differential_callback) {
    pressure_differential_callback(0, (32768 + msg_up->pressure_differential));
  }

  if (msg_up->valid.adc) {
    if (adc_callback) {
      adc_callback(msg_up->adc.channels);
    }
  }

  // Fill radio data
  if (msg_up->valid.rc && radio_control_callback) {
    radio_control.values[RADIO_ROLL] = msg_up->rc_roll;
    radio_control.values[RADIO_PITCH] = msg_up->rc_pitch;
    radio_control.values[RADIO_YAW] = msg_up->rc_yaw;
    radio_control.values[RADIO_THROTTLE] = msg_up->rc_thrust;
    radio_control.values[RADIO_MODE] = msg_up->rc_mode;
    radio_control.values[RADIO_KILL] = msg_up->rc_kill;
    radio_control.values[RADIO_GEAR] = msg_up->rc_gear;
    radio_control.values[RADIO_AUX2] = msg_up->rc_aux2;
    radio_control.values[RADIO_AUX3] = msg_up->rc_aux3;
    radio_control_callback();
  }
  // always fill status, it may change even when in the case when there is no new data
  radio_control.status = msg_up->rc_status;

  // Fill IMU data
  imuFloat.gyro.p = RATE_FLOAT_OF_BFP(msg_up->gyro.p);
  imuFloat.gyro.q = RATE_FLOAT_OF_BFP(msg_up->gyro.q);
  imuFloat.gyro.r = RATE_FLOAT_OF_BFP(msg_up->gyro.r);

  imuFloat.accel.x = ACCEL_FLOAT_OF_BFP(msg_up->accel.x);
  imuFloat.accel.y = ACCEL_FLOAT_OF_BFP(msg_up->accel.y);
  imuFloat.accel.z = ACCEL_FLOAT_OF_BFP(msg_up->accel.z);

  imuFloat.mag.x = MAG_FLOAT_OF_BFP(msg_up->mag.x);
  imuFloat.mag.y = MAG_FLOAT_OF_BFP(msg_up->mag.y);
  imuFloat.mag.z = MAG_FLOAT_OF_BFP(msg_up->mag.z);

  imuFloat.sample_count = msg_up->imu_tick;

  if (msg_up->valid.imu) {
    rdyb_booz_imu_update(&imuFloat);
  }
}

static void passthrough_down_fill(struct AutopilotMessagePTDown *msg_down)
{
  for (int i = 0; i < LISA_PWM_OUTPUT_NB; i++) {
    msg_down->pwm_outputs_usecs[i] = actuators[i];
  }
}

void spi_ap_link_periodic()
{
  static struct AutopilotMessageCRCFrame msg_up;
  static struct AutopilotMessageCRCFrame msg_down;
  uint8_t crc_valid;

  passthrough_down_fill(&msg_down.payload.msg_down);

  // SPI transcieve
  spi_link_send(&msg_down, sizeof(struct AutopilotMessageCRCFrame), &msg_up, &crc_valid);

  if (crc_valid) {
    passthrough_up_parse(&msg_up.payload.msg_up);
  }
}
