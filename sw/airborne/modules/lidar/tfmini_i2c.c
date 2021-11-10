/*
 * Copyright (C) 2020 OpenUAS <noreply@openuas.org>
 * Thanks to  Michal Podhradsky for SF11 work done
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


/** @file modules/lidar/tfmini_i2c.h
 *  @brief Driver for the TFMini ranging device when used via I2C bus
 */

#include "generated/airframe.h"
#include "modules/lidar/tfmini_i2c.h"
#include "modules/core/abi.h"

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#include "pprzlink/messages.h"
#endif

#ifdef SENSOR_SYNC_SEND
#include "subsystems/datalink/downlink.h"
#endif

#ifdef SITL
#include "state.h"
#endif

#ifndef AGL_LIDAR_TFMINI_I2C_ID
#define AGL_LIDAR_TFMINI_I2C_ID AGL_LIDAR_TFMINI_ID
#endif

// Check if an I2C device is selected
#ifndef TFMINI_I2C_DEV
#error TFMINI_I2C_DEV needs to be defined
#endif
PRINT_CONFIG_VAR(TFMINI_I2C_DEV)

/* TODO: not all possible sensor features are implemented(Yet), only basic functionality to get distance */

// Default base address on 8 bits
#ifndef TFMINI_I2C_ADDR
#define TFMINI_I2C_ADDR 0x10
#endif

#define TFMINI_I2C_REG_ADDR  0x26
#define TFMINI_I2C_REG_VAL   0x10
#define TFMINI_I2C_READ_ADDR 0x10

#define DISTANCE_MODE_REG_ADDR 0x50
#define DISTANCE_MODE_SHORT    0x00   // for 0.3-2m
#define DISTANCE_MODE_MIDDLE   0x03   // for 0.5-5m
#define DISTANCE_MODE_LONG     0x07   // for 1-12m

#define DETECTION_PATTERN_REG_ADDR  0x51
#define AUTOMATIC_DETECTION_PATTERN 0x00
#define FIXED_DETECTION_PATTERN     0x01

#define SETTING_OF_RANGE_LIMIT_REG_ADDR 0x55
#define RANGE_LIMIT_DISABLED 0x00
#define RANGE_LIMIT_ENABLED  0x01

#define RANGE_OUTPUT_LIMIT_THRESHOLD_REG_ADDR_L 0x56
#define RANGE_OUTPUT_LIMIT_THRESHOLD_REG_ADDR_H 0x57

//Settable 0x0000~0xFFFF
//Unit: mm12000(DEC)
#define LOWER_LIMIT_OF_SIGNAL_STRENGTH_THRESHOLD_REG_ADDR_L 0x58
#define LOWER_LIMIT_OF_SIGNAL_STRENGTH_THRESHOLD_REG_ADDR_H 0x59

//settable 0x0000~0xFFFF
//When Strength is lower than that value, the distance output is 0xFFFF, and is not credible to be used as flag.20(DEC)
#define UPPER_LIMIT_OF_SIGNAL_STRENGTH_THRESHOLD_REG_ADDR_L 0x5A
#define UPPER_LIMIT_OF_SIGNAL_STRENGTH_THRESHOLD_REG_ADDR_H 0x5B

//Settable 0x0000~0xFFFF
#define OUTPUT_VALUE_OF_SIGNAL_STRENGTH_THRESHOLD_AT_THE_HIGHEST_POINT_REG_ADDR_L 0x5C
#define OUTPUT_VALUE_OF_SIGNAL_STRENGTH_THRESHOLD_AT_THE_HIGHEST_POINT_REG_ADDR_H 0x5D

//Settable 0x0000~0xFFFF
#define UNIT_OF_DISTANCE_REG_ADDR 0x66
#define UNIT_OF_DISTANCE_MM   0x00 //output millimeter (mm)
#define UNIT_OF_DISTANCE_CM   0x01 // outout centimeter (cm)

#define SLAVE_ADDRESS_REG_ADDR 0x26 //settable 0x10~0x78  default 0x10

#define TRIGGER_MODE_REG_ADDR 0x27
#define USE_EXTERNAL_TRIGGER  0 // max100Hz  default 0
#define EXTERNAL_TRIGGER_REG_ADDR 0x01 //set to 0x01  Command for one single measurement

#define DEVICE_RESET_ADDR 0x70 //setable 0x02  //All settings are reset to the default(excluding slave address and trigger mode)

/// Ranger offset value for what considered the distance should be zero, e.g. high landing gear to tarmac 60cm still could be considerd zero
#ifndef TFMINI_I2C_OFFSET
#define TFMINI_I2C_OFFSET 0.0f
#endif

/// The minimum range for the device to be able to measure
#ifndef TFMINI_I2C_MIN_RANGE
#define TFMINI_I2C_MIN_RANGE 0.3f //Default for TFMini is 30cm(old firmware?), for TFMini Plus 10cm, sensors cannot measure a range closer than that
#endif

/// The maximum range for the device to be able to measure
#ifndef TFMINI_I2C_MAX_RANGE
#define TFMINI_I2C_MAX_RANGE 6.0f //Reasonable default for it is maximum value for both inside and outside lighting conditions in day environment
#endif

struct TFMiniI2C tfmini_i2c;

/**
 * Send measured value and status information so it can be read back in e.g. log file for debugging
 */
static void tfmini_i2c_send_lidar(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t trans_status = tfmini_i2c.trans.status;
  pprz_msg_send_LIDAR(trans, dev, AC_ID,
                      &tfmini_i2c.dist,
                      &tfmini_i2c.status,
                      &trans_status);
}

/**
 * Set the default values at initialization
 */
void tfmini_i2c_init(void)
{
  tfmini_i2c.trans.status = I2CTransDone;
  tfmini_i2c.addr = TFMINI_I2C_ADDR;

  tfmini_i2c.raw_dist = 0;
  tfmini_i2c.strength = 0; //Not implemented
  tfmini_i2c.dist = 0.0f;
  tfmini_i2c.offset = TFMINI_I2C_OFFSET;
  tfmini_i2c.update_agl = USE_TFMINI_I2C_AGL;
  tfmini_i2c.compensate_rotation = TFMINI_I2C_COMPENSATE_ROTATION;

  tfmini_i2c.status = TFMINI_I2C_ACQUIRE;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LIDAR, tfmini_i2c_send_lidar);
#endif
}

/**
 * Ranger event function
 * Basically just check the progress of the transation
 * to prevent overruns during high speed operation
 * (ie. polling the ranger at >100Hz)
 */
void tfmini_i2c_event(void)
{
  switch (tfmini_i2c.trans.status) {
    case I2CTransPending:
      // wait and do nothing
      break;
    case I2CTransRunning:
      // wait and do nothing
      break;
    case I2CTransSuccess:
      // set to done
      tfmini_i2c.trans.status = I2CTransDone;
      break;
    case I2CTransFailed:
      // set to done
      tfmini_i2c.trans.status = I2CTransDone;
      break;
    case I2CTransDone:
      // do nothing
      break;
    default:
      break;
  }
}

/**
 * 
 * Get the ranger current distance value
 */
void tfmini_i2c_periodic(void)
{

#ifndef SITL
  switch (tfmini_i2c.status) {
    case TFMINI_I2C_ACQUIRE:
      if (tfmini_i2c.trans.status == I2CTransDone) {
        tfmini_i2c.trans.buf[0] = 0x01; // sets register pointer to results register
        tfmini_i2c.trans.buf[1] = 0x02;
        tfmini_i2c.trans.buf[2] = 0x07;
        if (i2c_blocking_transceive(&TFMINI_I2C_DEV, &tfmini_i2c.trans, tfmini_i2c.addr, 3, 7)) {
          tfmini_i2c.status = TFMINI_I2C_PARSE;
        }
      };  //fallthrough
    case TFMINI_I2C_PARSE: {
      tfmini_i2c.raw_dist = (uint16_t)((tfmini_i2c.trans.buf[3] << 8) | tfmini_i2c.trans.buf[2]);
      uint32_t now_ts = get_sys_time_usec();

      tfmini_i2c.dist = ((float)(tfmini_i2c.raw_dist))*0.01f;

      Bound(tfmini_i2c.dist, (float)TFMINI_I2C_MIN_RANGE, (float)TFMINI_I2C_MAX_RANGE);

      tfmini_i2c.dist = tfmini_i2c.dist - tfmini_i2c.offset;

      //Compensate AGL measurement for body rotation
      if (tfmini_i2c.compensate_rotation) {
        float phi = stateGetNedToBodyEulers_f()->phi;
        float theta = stateGetNedToBodyEulers_f()->theta;
        float gain = (float)fabs((double)(cosf(phi) * cosf(theta)));
        tfmini_i2c.dist = tfmini_i2c.dist * gain;
      }

      // Datasheet states FFFF is the output if a valid distance value could not be aquired but depend on not implemented on setting
      //if (tfmini.dist != 0xFFFF) {
        if (tfmini_i2c.update_agl) {
          AbiSendMsgAGL(AGL_LIDAR_TFMINI_I2C_ID, now_ts, tfmini_i2c.dist);
        }
      //}

      tfmini_i2c.status = TFMINI_I2C_ACQUIRE;
      break;
    }
    default:
      break;
  }
#else // SITL
tfmini_i2c.dist = stateGetPositionEnu_f()->z;
#endif // SITL
}

//Sonar message can be misused used to debug raw and scaled
void tfmini_i2c_downlink(void)
{
#ifdef SENSOR_SYNC_SEND
  DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &tfmini_i2c.raw_dist, &tfmini_i2c.dist);
#endif
}
