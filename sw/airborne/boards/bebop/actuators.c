/*
 *
 * Copyright (C) 2014-2015 Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file boards/bebop/actuators.c
 * Actuator driver for the bebop and bebop 2
 */

#include "subsystems/actuators.h"
#include "subsystems/electrical.h"
#include "actuators.h"
#include "led_hw.h"
#include "autopilot.h"
#include "subsystems/abi.h"

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/stabilization.h"

static void send_bebop_actuators(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_BEBOP_ACTUATORS(trans, dev, AC_ID,
                                &stabilization_cmd[COMMAND_THRUST],
                                &stabilization_cmd[COMMAND_ROLL],
                                &stabilization_cmd[COMMAND_PITCH],
                                &stabilization_cmd[COMMAND_YAW],
                                &actuators_bebop.rpm_ref[0],
                                &actuators_bebop.rpm_ref[1],
                                &actuators_bebop.rpm_ref[2],
                                &actuators_bebop.rpm_ref[3],
                                &actuators_bebop.rpm_obs[0],
                                &actuators_bebop.rpm_obs[1],
                                &actuators_bebop.rpm_obs[2],
                                &actuators_bebop.rpm_obs[3]);
}
#endif

uint32_t led_hw_values;
struct ActuatorsBebop actuators_bebop;
static uint8_t actuators_bebop_checksum(uint8_t *bytes, uint8_t size);

void actuators_bebop_init(void)
{
  /* Initialize the I2C connection */
  actuators_bebop.i2c_trans.slave_addr = ACTUATORS_BEBOP_ADDR;
  actuators_bebop.i2c_trans.status = I2CTransDone;
  actuators_bebop.led = 0;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_BEBOP_ACTUATORS, send_bebop_actuators);
#endif
}

void actuators_bebop_commit(void)
{
  // Receive the status
  actuators_bebop.i2c_trans.buf[0] = ACTUATORS_BEBOP_GET_OBS_DATA;
  i2c_blocking_transceive(&i2c1, &actuators_bebop.i2c_trans, actuators_bebop.i2c_trans.slave_addr, 1, 13);

  // Update status
  electrical.vsupply = (float)(actuators_bebop.i2c_trans.buf[9] + (actuators_bebop.i2c_trans.buf[8] << 8)) / 1000.f;
  // The 15th bit contains saturation information, so it needs to be removed to get the rpm
  actuators_bebop.rpm_obs[0] = (actuators_bebop.i2c_trans.buf[1] + (actuators_bebop.i2c_trans.buf[0] << 8)) & ~(1<<15);
  actuators_bebop.rpm_obs[1] = (actuators_bebop.i2c_trans.buf[3] + (actuators_bebop.i2c_trans.buf[2] << 8)) & ~(1<<15);
  actuators_bebop.rpm_obs[2] = (actuators_bebop.i2c_trans.buf[5] + (actuators_bebop.i2c_trans.buf[4] << 8)) & ~(1<<15);
  actuators_bebop.rpm_obs[3] = (actuators_bebop.i2c_trans.buf[7] + (actuators_bebop.i2c_trans.buf[6] << 8)) & ~(1<<15);

  // When detected a suicide
  actuators_bebop.i2c_trans.buf[10] = actuators_bebop.i2c_trans.buf[10] & 0x7;
  if (actuators_bebop.i2c_trans.buf[11] == 2 && actuators_bebop.i2c_trans.buf[10] != 1) {
    autopilot_set_motors_on(FALSE);
  }

  // Start the motors
  if (actuators_bebop.i2c_trans.buf[10] != 4 && actuators_bebop.i2c_trans.buf[10] != 2 && autopilot_get_motors_on()) {
    // Reset the error
    actuators_bebop.i2c_trans.buf[0] = ACTUATORS_BEBOP_CLEAR_ERROR;
    i2c_blocking_transmit(&i2c1, &actuators_bebop.i2c_trans, actuators_bebop.i2c_trans.slave_addr, 1);

    // Start the motors
    actuators_bebop.i2c_trans.buf[0] = ACTUATORS_BEBOP_START_PROP;
#if BEBOP_VERSION2
    // For Bebop version 2 some motors are reversed (FIXME: test final version)
    actuators_bebop.i2c_trans.buf[1] = 0b00001010;
#else
    actuators_bebop.i2c_trans.buf[1] = 0b00000101;
#endif
    i2c_blocking_transmit(&i2c1, &actuators_bebop.i2c_trans, actuators_bebop.i2c_trans.slave_addr, 2);
  }
  // Stop the motors
  else if (actuators_bebop.i2c_trans.buf[10] == 4 && !autopilot_get_motors_on()) {
    actuators_bebop.i2c_trans.buf[0] = ACTUATORS_BEBOP_STOP_PROP;
    i2c_blocking_transmit(&i2c1, &actuators_bebop.i2c_trans, actuators_bebop.i2c_trans.slave_addr, 1);
  } else if (actuators_bebop.i2c_trans.buf[10] == 4 && autopilot_get_motors_on()) {
    // Send the commands
    actuators_bebop.i2c_trans.buf[0] = ACTUATORS_BEBOP_SET_REF_SPEED;
    actuators_bebop.i2c_trans.buf[1] = actuators_bebop.rpm_ref[0] >> 8;
    actuators_bebop.i2c_trans.buf[2] = actuators_bebop.rpm_ref[0] & 0xFF;
    actuators_bebop.i2c_trans.buf[3] = actuators_bebop.rpm_ref[1] >> 8;
    actuators_bebop.i2c_trans.buf[4] = actuators_bebop.rpm_ref[1] & 0xFF;
    actuators_bebop.i2c_trans.buf[5] = actuators_bebop.rpm_ref[2] >> 8;
    actuators_bebop.i2c_trans.buf[6] = actuators_bebop.rpm_ref[2] & 0xFF;
    actuators_bebop.i2c_trans.buf[7] = actuators_bebop.rpm_ref[3] >> 8;
    actuators_bebop.i2c_trans.buf[8] = actuators_bebop.rpm_ref[3] & 0xFF;
    actuators_bebop.i2c_trans.buf[9] = 0x00; //UNK enable security?
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
    actuators_bebop.i2c_trans.buf[10] = actuators_bebop_checksum((uint8_t *)actuators_bebop.i2c_trans.buf, 9);
#pragma GCC diagnostic pop
    i2c_blocking_transmit(&i2c1, &actuators_bebop.i2c_trans, actuators_bebop.i2c_trans.slave_addr, 11);
  }

  // Update the LEDs
  if (actuators_bebop.led != (led_hw_values & 0x3)) {
    actuators_bebop.i2c_trans.buf[0] = ACTUATORS_BEBOP_TOGGLE_GPIO;
    actuators_bebop.i2c_trans.buf[1] = (led_hw_values & 0x3);
    i2c_blocking_transmit(&i2c1, &actuators_bebop.i2c_trans, actuators_bebop.i2c_trans.slave_addr, 2);

    actuators_bebop.led = led_hw_values & 0x3;
  }
  // Send ABI message
  AbiSendMsgRPM(RPM_SENSOR_ID, actuators_bebop.rpm_obs, 4);
}

static uint8_t actuators_bebop_checksum(uint8_t *bytes, uint8_t size)
{
  uint8_t checksum = 0;
  for (int i = 0; i < size; i++) {
    checksum = checksum ^ bytes[i];
  }

  return checksum;
}
