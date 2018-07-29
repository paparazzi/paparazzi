/*
 *
 * Copyright (C) 2014-2015 Freek van Tienen <freek.v.tienen@gmail.com>
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file boards/disco/actuators.c
 * Actuator driver for the Parrot Disco
 *
 * Disco plane is using the same ESC (I2C) as a Parrot Bebop for its motor
 * and Pwm_sysfs linux driver for the PWM outputs
 *
 * Some part of this code is coming from the APM Disco and Bebop drivers
 */

#include "subsystems/actuators.h"
#include "subsystems/electrical.h"
#include "actuators.h"
#include "autopilot.h"
#include "subsystems/abi.h"
#include <endian.h>
#include <string.h>

#include <stdio.h>

/**
 * private observation structure
 */
struct __attribute__((__packed__)) disco_bldc_obs {
  uint16_t rpm;
  uint16_t batt_mv;
  uint8_t  status;
  uint8_t  error;
  uint8_t  motors_err;
  uint8_t  temp;
  /* bit 0 indicates an overcurrent on the RC receiver port when high
   * bits #1-#6 indicate an overcurrent on the #1-#6 PPM servos
   */
  uint8_t  overrcurrent;
  uint8_t  checksum;
} obs_data;

/**
 * Internal mapping of the PWM with output index
 * servo rail 1 <-> linux pwm_4
 * servo rail 2 <-> linux pwm_5
 * servo rail 3 <-> linux pwm_6
 * servo rail 4 <-> linux pwm_1
 * servo rail 5 <-> linux pwm_2
 * servo rail 6 <-> linux pwm_3
 */
static uint8_t disco_channels[] = { 4, 5, 6, 1, 2, 3 };

#define DISCO_BLDC_STATUS_STOPPED   1
#define DISCO_BLDC_STATUS_RAMPUP    2
#define DISCO_BLDC_STATUS_RUNNING   4
#define DISCO_BLDC_STATUS_RAMPDOWN  5

// motor start threshold in RPM
// RPM range on disco is [1000, 12500]
// start and 1100
#define DISCO_BLDC_START_MOTOR_THRESHOLD 1100

struct ActuatorsDisco actuators_disco;
static uint8_t actuators_disco_checksum(uint8_t *bytes, uint8_t size);

void actuators_disco_init(void)
{
  /* Initialize the I2C connection */
  actuators_disco.i2c_trans.slave_addr = ACTUATORS_DISCO_ADDR;
  actuators_disco.i2c_trans.status = I2CTransDone;
  actuators_disco.motor_rpm = 0;
  int i = 0;
  for (i = 0; i < ACTUATORS_DISCO_PWM_NB; i++) {
    pwm_sysfs_init(&actuators_disco.pwm[i], "/sys/class/pwm", "export", "run", "duty_ns", "period_ns", disco_channels[i]);
  }
}

void actuators_disco_set(uint8_t idx, uint16_t val)
{
  if (idx == ACTUATORS_DISCO_MOTOR_IDX) {
    actuators_disco.motor_rpm = val;
  } else if (idx > ACTUATORS_DISCO_PWM_NB) {
    // wrong index, do nothing
  } else {
    // val is a PWM value in ms, convert to ns
    pwm_sysfs_set_duty(&actuators_disco.pwm[idx-1], val * 1000);
  }
}

void actuators_disco_commit(void)
{
  // Handle PWM outputs
  // already done in set function (FIXME ?)

  // Handle motor speed controller

  // Receive the status
  actuators_disco.i2c_trans.buf[0] = ACTUATORS_DISCO_GET_OBS_DATA;
  i2c_blocking_transceive(&i2c1, &actuators_disco.i2c_trans, actuators_disco.i2c_trans.slave_addr, 1, sizeof(obs_data));
  // copy data from buffer
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
  memcpy(&obs_data, (uint8_t*)actuators_disco.i2c_trans.buf, sizeof(obs_data));
#pragma GCC diagnostic pop

  // Update status
  electrical.vsupply = be16toh(obs_data.batt_mv) / 100;
  // extract 'rpm saturation bit'
  actuators_disco.rpm_saturated = (obs_data.rpm & (1 << 7)) ? true : false;
  // clear 'rpm saturation bit' and fix endianess
  obs_data.rpm &= (uint16_t)(~(1 << 7));
  obs_data.rpm = be16toh(obs_data.rpm);
  if (obs_data.rpm == 0) {
    actuators_disco.rpm_saturated = false;
  }

  // When detected a suicide
  uint8_t bldc_status = obs_data.status & 0x07;
  if (obs_data.error == 2 && bldc_status != DISCO_BLDC_STATUS_STOPPED) {
    autopilot_set_kill_throttle(true); //FIXME: make behaviour definable low flying should not stop it
  }

  // Start the motors
  if (bldc_status != DISCO_BLDC_STATUS_RUNNING &&
      bldc_status != DISCO_BLDC_STATUS_RAMPUP && //FIXME also on rampdown?
      actuators_disco.motor_rpm > DISCO_BLDC_START_MOTOR_THRESHOLD) {
    // Reset the error
    actuators_disco.i2c_trans.buf[0] = ACTUATORS_DISCO_CLEAR_ERROR;
    i2c_blocking_transmit(&i2c1, &actuators_disco.i2c_trans, actuators_disco.i2c_trans.slave_addr, 1);

    // Start the motors
    actuators_disco.i2c_trans.buf[0] = ACTUATORS_DISCO_START_PROP;
    i2c_blocking_transmit(&i2c1, &actuators_disco.i2c_trans, actuators_disco.i2c_trans.slave_addr, 1);
  }
  // Stop the motors
  else if ((bldc_status == DISCO_BLDC_STATUS_RUNNING || bldc_status == DISCO_BLDC_STATUS_RAMPUP) &&
      actuators_disco.motor_rpm < DISCO_BLDC_START_MOTOR_THRESHOLD) {
    actuators_disco.i2c_trans.buf[0] = ACTUATORS_DISCO_STOP_PROP;
    i2c_blocking_transmit(&i2c1, &actuators_disco.i2c_trans, actuators_disco.i2c_trans.slave_addr, 1);
  } else if (bldc_status == DISCO_BLDC_STATUS_RUNNING) {
    // Send the commands
    actuators_disco.i2c_trans.buf[0] = ACTUATORS_DISCO_SET_REF_SPEED;
    actuators_disco.i2c_trans.buf[1] = actuators_disco.motor_rpm >> 8;
    actuators_disco.i2c_trans.buf[2] = actuators_disco.motor_rpm & 0xFF;
    actuators_disco.i2c_trans.buf[3] = 0x00; // enable security
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
    actuators_disco.i2c_trans.buf[4] = actuators_disco_checksum((uint8_t *)actuators_disco.i2c_trans.buf, 3);
#pragma GCC diagnostic pop
    i2c_blocking_transmit(&i2c1, &actuators_disco.i2c_trans, actuators_disco.i2c_trans.slave_addr, 11);
  }

  // Send ABI message
  AbiSendMsgRPM(RPM_SENSOR_ID, &actuators_disco.rpm_obs, 1);//FIXME & or not
}

static uint8_t actuators_disco_checksum(uint8_t *bytes, uint8_t size)
{
  uint8_t checksum = 0;
  for (int i = 0; i < size; i++) {
    checksum = checksum ^ bytes[i];
  }

  return checksum;
}
