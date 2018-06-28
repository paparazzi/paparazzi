/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * @file "modules/actuators/actuators_md25.c"
 * @author Gautier Hattenberger
 * Driver for the MD25 rover controller board
 */

#include "subsystems/actuators.h"
#include "modules/actuators/actuators_md25.h"

// Registers
#define MD25_REG_SPEED1           0x00
#define MD25_REG_SPEED2           0x01
#define MD25_REG_ENCODER1         0x02
#define MD25_REG_ENCODER2         0x06
#define MD25_REG_BAT              0x0A
#define MD25_REG_CURRENT1         0x0B
#define MD25_REG_CURRENT2         0x0C
#define MD25_REG_SOFT_REV         0x0D
#define MD25_REG_ACCEL_RATE       0x0E
#define MD25_REG_MODE             0x0F
#define MD25_REG_COMMAND          0x10
// Commands
#define MD25_CMD_RESET_ENCODERS   0x20
#define MD25_CMD_NO_SPEED_REGUL   0x30
#define MD25_CMD_SPEED_REGUL      0x31
#define MD25_CMD_NO_MOTOR_TIMEOUT 0x32
#define MD25_CMD_MOTOR_TIMEOUT    0x33

// Modes
#define MD25_MODE0 0 // motor1 speed, motor2 speed [0, 255] (default)
#define MD25_MODE1 1 // motor1 speed, motor2 speed [-128, 127]
#define MD25_MODE2 2 // speed, turn [0, 255]
#define MD25_MODE3 3 // speed, turn [-128, 127]

// default control mode
#ifndef ACTUATORS_MD25_MODE
#define ACTUATORS_MD25_MODE MD25_MODE0
#endif

// default accel rate
#ifndef ACTUATORS_MD25_ACCEL_RATE
#define ACTUATORS_MD25_ACCEL_RATE 5
#endif

// 7 bits I2C address
#ifndef ACTUATORS_MD25_I2C_ADDR
#define ACTUATORS_MD25_I2C_ADDR 0xB0
//#define ACTUATORS_MD25_I2C_ADDR 0x58
#endif

PRINT_CONFIG_VAR(ACTUATORS_MD25_DEV)

struct ActuatorsMD25 actuators_md25;

void actuators_md25_init(void)
{
  actuators_md25.mode = ACTUATORS_MD25_MODE;
  actuators_md25.bat = 0;
  actuators_md25.cmds[0] = 0;
  actuators_md25.cmds[1] = 0;
  actuators_md25.encoders[0] = 0;
  actuators_md25.encoders[1] = 0;
  actuators_md25.mode = ACTUATORS_MD25_MODE;
  actuators_md25.accel_rate = ACTUATORS_MD25_ACCEL_RATE;
  actuators_md25.initialized = false;
  actuators_md25.trans_cmd.status = I2CTransDone;
  actuators_md25.trans_sensors.status = I2CTransDone;
}

void actuators_md25_periodic(void)
{
  if (actuators_md25.trans_sensors.status == I2CTransDone) {
    if (!actuators_md25.initialized) {
      // send accel rate and mode configuration
      actuators_md25.trans_sensors.buf[0] = MD25_REG_ACCEL_RATE;
      actuators_md25.trans_sensors.buf[1] = actuators_md25.accel_rate;
      actuators_md25.trans_sensors.buf[2] = actuators_md25.mode;
      i2c_transmit(&(ACTUATORS_MD25_DEV), &actuators_md25.trans_sensors, ACTUATORS_MD25_I2C_ADDR, 3);
    }
    else {
      // read sensors: enc1, enc1, bat, cur1, cur2
      actuators_md25.trans_sensors.buf[0] = MD25_REG_ENCODER1;
      i2c_transceive(&(ACTUATORS_MD25_DEV), &actuators_md25.trans_sensors, ACTUATORS_MD25_I2C_ADDR, 1, 11);
    }
  }
}

void actuators_md25_set(void)
{
  if (actuators_md25.initialized && actuators_md25.trans_cmd.status == I2CTransDone) {
    actuators_md25.trans_cmd.buf[0] = MD25_REG_SPEED1;
    actuators_md25.trans_cmd.buf[1] = actuators_md25.cmds[0];
    actuators_md25.trans_cmd.buf[2] = actuators_md25.cmds[1];
    i2c_transmit(&(ACTUATORS_MD25_DEV), &actuators_md25.trans_cmd, ACTUATORS_MD25_I2C_ADDR, 3);
  }
}

#define Int32FromBuf(_buf,_idx) ((int32_t)(((uint32_t)_buf[_idx]<<24) | ((uint32_t)_buf[_idx+1]<<16) | ((uint32_t)_buf[_idx+2]<<8) | _buf[_idx+3]))

void actuators_md25_event(void)
{
  // commands
  if (actuators_md25.trans_cmd.status == I2CTransSuccess) {
    actuators_md25.trans_cmd.status = I2CTransDone;
  }
  else if (actuators_md25.trans_cmd.status == I2CTransFailed) {
    // TODO handle or report error
    actuators_md25.trans_cmd.status = I2CTransDone;
  }

  // sensors
  if (actuators_md25.trans_sensors.status == I2CTransSuccess) {
    if (actuators_md25.initialized) {
      actuators_md25.encoders[0] = Int32FromBuf(actuators_md25.trans_sensors.buf, 0);
      actuators_md25.encoders[1] = Int32FromBuf(actuators_md25.trans_sensors.buf, 4);
      actuators_md25.bat = actuators_md25.trans_sensors.buf[8];
      actuators_md25.current[0] = actuators_md25.trans_sensors.buf[9];
      actuators_md25.current[1] = actuators_md25.trans_sensors.buf[10];
      actuators_md25.trans_sensors.status = I2CTransDone;
    }
    else {
      actuators_md25.trans_sensors.status = I2CTransDone;
      actuators_md25.initialized = true;
    }
  }
  else if (actuators_md25.trans_sensors.status == I2CTransFailed) {
    // TODO handle or report error
    actuators_md25.trans_sensors.status = I2CTransDone;
  }
}

