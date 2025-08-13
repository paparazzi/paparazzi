/*
 * Copyright (C) 2023 OpenUAS <noreply@openuas.org>
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


/** @file modules/sonar/sonar_i2c.h
 *  @brief Driver for an sonar rangfinder sensor when used via I2C bus
 */

#include "generated/airframe.h"
#include "modules/sonar/sonar_i2c.h"
#include "modules/core/abi.h"
#if defined(SITL) || (defined(SONAR_I2C_COMPENSATE_ROTATION) && (SONAR_I2C_COMPENSATE_ROTATION == 1))
#include "state.h"
#endif
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "pprzlink/messages.h"
#endif

#ifdef MODULE_SONAR_I2C_SYNC_SEND
#include "modules/datalink/downlink.h"
#endif

#ifndef SONAR_I2C_AGL_ID
#define SONAR_I2C_AGL_ID AGL_SONAR_I2C_ID
#endif

// Check if an I2C device is selected
#ifndef SONAR_I2C_DEV
#error SONAR_I2C_DEV needs to be defined
#endif
PRINT_CONFIG_VAR(SONAR_I2C_DEV)

// Default base address on 8 bits
#ifndef SONAR_I2C_ADDR
#define SONAR_I2C_ADDR 0xE0 // Equals 0x70 if notated as 7bit address
#endif

#define READ_MODE_SINGLE 0x51 // Deducted from sparse random information

// The minimum and maximum range what we want in our output e.g. sensor can do 7.2m but we only want to get dat less than 4m set MAX_RANGE to 4.0

/// The minimum chosen distance for the device to be give readings
#ifndef SONAR_I2C_MIN_RANGE
#define SONAR_I2C_MIN_RANGE 0.24f //Default for Sonar is 0.24m since many sonar sensors cannot measure a range closer than that reliably
#endif

/// The maximum chosen distance for the device to be give readings
#ifndef SONAR_I2C_MAX_RANGE
#define SONAR_I2C_MAX_RANGE 4.0f //Reasonable default value in meters with still usable readings for it is maximum value for common sonar sensors
#endif

/// Rangefinder distance offset value for what should be considered zero distance, e.g. high landing gear of 1.1m to tarmac still could be considered zero
#ifndef SONAR_I2C_OFFSET
#define SONAR_I2C_OFFSET 0.0f
#endif

/// Send AGL data over ABI
#ifndef USE_SONAR_I2C_AGL
#define USE_SONAR_I2C_AGL 0
#endif

/// Filter the raw measuread sonar data
#ifndef SONAR_I2C_USE_FILTER
#define SONAR_I2C_USE_FILTER 0 // Enable filtering for sonar per default is best, sonars tend to give noisy spiking readings
#endif

#ifdef SONAR_I2C_USE_FILTER
#include "filters/median_filter.h"
///The amount of sensor samples to keep in the median filter buffer
#ifndef SONAR_I2C_MEDIAN_SIZE
#define SONAR_I2C_MEDIAN_SIZE 7 // Default median filter length of 7 is a good fit for most sonar sensors
#endif
//struct MedianFilterInt sonar_i2c_filter;
struct MedianFilterFloat sonar_i2c_filter;
#endif

#ifndef SONAR_I2C_COMPENSATE_ROTATION
#define SONAR_I2C_COMPENSATE_ROTATION 0
#endif

// Gain for sonar sensors to get from raw measuread value to meters
#ifndef SONAR_I2C_SCALE
#define SONAR_I2C_SCALE 0.000044f  //Experimentally determined gain for famous GY-US42V2 sensor, no datasheet remember ;)
#endif

struct SonarI2C sonar_i2c;

/**
 * Send measured value and status information so it can be read back in e.g. log file for debugging
 */
static void sonar_i2c_send_sonar(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SONAR(trans, dev, AC_ID, &sonar_i2c.raw, &sonar_i2c.distance);
}

/**
 * Set the default values at initialization
 */
void sonar_i2c_init(void)
{
  sonar_i2c.trans.status = I2CTransDone;
  sonar_i2c.addr = SONAR_I2C_ADDR;

  //Init with defaults that do not cause harm
  sonar_i2c.raw = 0; 
  sonar_i2c.distance = 0.0;
  sonar_i2c.update_agl = USE_SONAR_I2C_AGL;

  sonar_i2c.status = SONAR_I2C_REQ_DATA;

#ifdef SONAR_I2C_USE_FILTER
  init_median_filter_f(&sonar_i2c_filter, SONAR_I2C_MEDIAN_SIZE);
#endif

//#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SONAR, sonar_i2c_send_sonar);
//#endif

}

/**
 * Rangefinder event function
 * Basically just check the progress of the transation
 * to prevent overruns during high speed operation
 * (ie. polling the sensor at >50Hz)
 */
void sonar_i2c_event(void)
{
  switch (sonar_i2c.trans.status) {
    case I2CTransPending:
      // wait and do nothing
      break;
    case I2CTransRunning:
      // wait and do nothing
      break;
    case I2CTransSuccess:
      // set to done
      sonar_i2c.trans.status = I2CTransDone;
      break;
    case I2CTransFailed:
      // set to done
      sonar_i2c.trans.status = I2CTransDone;
      break;
    case I2CTransDone:
      // do nothing
      break;
    default:
      break;
  }
#if MODULE_SONAR_I2C_SYNC_SEND
  sonar_i2c_report();
#endif
}

/**
 *
 * Get the ranger current distance value
 */
void sonar_i2c_periodic(void)
{
#ifndef SITL
  switch (sonar_i2c.status) {

    //Blocking I2C Transceive did not work for some of those I2C devices, therefore a state machine to handle the I2C transactions
    case SONAR_I2C_REQ_DATA:
      if (sonar_i2c.trans.status == I2CTransDone) {
        sonar_i2c.trans.buf[0] = READ_MODE_SINGLE;
        if (i2c_transmit(&SONAR_I2C_DEV, &sonar_i2c.trans, sonar_i2c.addr, 1)) {
          sonar_i2c.status = SONAR_I2C_READ_DATA;
        }
      }
      break;
    case SONAR_I2C_READ_DATA:
      if (sonar_i2c.trans.status == I2CTransDone) {
        sonar_i2c.trans.buf[1] = 0;
        sonar_i2c.trans.buf[2] = 0;
        if (i2c_receive(&SONAR_I2C_DEV, &sonar_i2c.trans, sonar_i2c.addr, 2)) {
          sonar_i2c.status = SONAR_I2C_PARSE_DATA;
        }
        sonar_i2c.raw = (uint16_t)sonar_i2c.trans.buf[2]; // Debugging value, to see if we got a valid reading
        DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &sonar_i2c.raw, &sonar_i2c.distance);
      }
      break;
    case SONAR_I2C_PARSE_DATA: {
      sonar_i2c.raw = (uint16_t)((sonar_i2c.trans.buf[1] << 8) | sonar_i2c.trans.buf[2]);
      // Time of when measurement was taken, not when ABI message was send, tiny delay can occur between those two events
      uint32_t now_ts = get_sys_time_usec();

      // Convert the raw value to meters, optionally filter and apply the offset
      sonar_i2c.distance = (((float)(sonar_i2c.raw)) * SONAR_I2C_SCALE);
#ifdef SONAR_I2C_USE_FILTER
      sonar_i2c.distance = update_median_filter_f(&sonar_i2c_filter, sonar_i2c.distance);
#endif
      sonar_i2c.distance = sonar_i2c.distance - SONAR_I2C_OFFSET;
      //sonar_i2c.raw = 33; // Debugging value, to see if we got a valid reading
      //DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &sonar_i2c.raw, &sonar_i2c.distance);
      Bound(sonar_i2c.distance, (float)SONAR_I2C_MIN_RANGE, (float)SONAR_I2C_MAX_RANGE);

      // Compensate range measurement for body rotation
#if SONAR_I2C_COMPENSATE_ROTATION
      float phi = stateGetNedToBodyEulers_f()->phi;
      float theta = stateGetNedToBodyEulers_f()->theta;
      float gain = (float)fabs((double)(cosf(phi) * cosf(theta)));
      sonar_i2c.distance = sonar_i2c.distance * gain;
#endif

      // Send AGL message
      if (sonar_i2c.update_agl) {
        if(!isnan(sonar_i2c.distance)) {
          AbiSendMsgAGL(SONAR_I2C_AGL_ID, now_ts, sonar_i2c.distance);
        }
      }

      // Reset status as so to start reading new distance value again
      sonar_i2c.status = SONAR_I2C_REQ_DATA;
      break;
    }
    default:
      break;
  }

#else // SITL
  sonar_i2c.distance = stateGetPositionEnu_f()->z;
#endif // SITL
}

/**
 *
 * Option to send debug informative values over telemetry
 */
void sonar_i2c_report(void)
{
  DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &sonar_i2c.raw, &sonar_i2c.distance);
}
