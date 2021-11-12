/*
* Copyright (C) 2015 Jean-François Erdelyi, Gautier Hattenberger
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
* @file "modules/sensors/aoa_pwm.c"
* @author Jean-François Erdelyi
* @brief Angle of Attack sensor on PWM
*
* Driver for a PWM based angle of attack sensor
* A second sensor can be defined for the sideslip angle
* It is assumed that both sensors are the same, only
* sensitivity, offset and direction can differ.
*
* SENSOR, exemple : US DIGITAL MA3-P12-125-B
* @see http://www.usdigital.com/products/encoders/absolute/rotary/shaft/ma3
*/

#include "modules/sensors/aoa_pwm.h"
#include "mcu_periph/pwm_input.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "modules/core/abi.h"
#include "generated/airframe.h"

#if LOG_AOA
#include "modules/loggers/sdlog_chibios.h"
bool log_started;
#endif

/* Config parameters for angle of attack sensor (mandatory) */

#ifndef AOA_PWM_CHANNEL
#error "AOA_PWM_CHANNEL needs to be defined to use AOA_pwm module"
#endif

/// Default to a 12 bit PWM sensor
#ifndef AOA_PWM_PERIOD
#define AOA_PWM_PERIOD 4096
#endif
/// Some sensor may need an initial PWM offset (1 usec in the case of an MA3 sensor)
#ifndef AOA_PWM_OFFSET
#define AOA_PWM_OFFSET 1
#endif
/// Default offset value (assuming 0 AOA is in the middle of the range)
#ifndef AOA_ANGLE_OFFSET
#define AOA_ANGLE_OFFSET M_PI
#endif
/// Default extra offset that can be ajusted from settings
#ifndef AOA_OFFSET
#define AOA_OFFSET 0.0f
#endif
/// Default filter value
#ifndef AOA_FILTER
#define AOA_FILTER 0.0f
#endif
/// Default sensitivity (2*pi on a PWM of period AOA_PWM_PERIOD)
#ifndef AOA_SENS
#define AOA_SENS ((2.0f*M_PI)/AOA_PWM_PERIOD)
#endif
// Set AOA_REVERSE to TRUE to change rotation direction
#if AOA_REVERSE
#define AOA_SIGN -1
#else
#define AOA_SIGN 1
#endif
// Enable telemetry report
#ifndef SEND_SYNC_AOA
#define SEND_SYNC_AOA TRUE
#endif
// Enable SD card logging
#ifndef LOG_AOA
#define LOG_AOA FALSE
#endif

struct Aoa_Pwm aoa_pwm;


/* Config parameters for sideslip angle sensor (optional) */

#if defined USE_SIDESLIP && !(defined SSA_PWM_CHANNEL)
#error "SSA_PWM_CHANNEL needs to be defined to use sideslip sensor"
#endif

// Default extra offset that can be ajusted from settings
#ifndef SSA_OFFSET
#define SSA_OFFSET 0.0f
#endif
// Default filter value
#ifndef SSA_FILTER
#define SSA_FILTER 0.0f
#endif
// Default sensitivity (2*pi on a PWM of period AOA_PWM_PERIOD)
#ifndef SSA_SENS
#define SSA_SENS ((2.0f*M_PI)/AOA_PWM_PERIOD)
#endif
// Set SSA_REVERSE to TRUE to change rotation direction
#if SSA_REVERSE
#define SSA_SIGN -1
#else
#define SSA_SIGN 1
#endif
struct Aoa_Pwm ssa_pwm;


/* telemetry */
enum Aoa_Type aoa_send_type;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_aoa(struct transport_tx *trans, struct link_device *dev)
{
  // FIXME use a second message, more fields or add a sensor ID to send sideslip
  switch (aoa_send_type) {
    case SEND_TYPE_SIDESLIP:
      pprz_msg_send_AOA(trans, dev, AC_ID, &ssa_pwm.raw, &ssa_pwm.angle);
      break;
    case SEND_TYPE_AOA:
    default:
      pprz_msg_send_AOA(trans, dev, AC_ID, &aoa_pwm.raw, &aoa_pwm.angle);
      break;
  }
}

#endif

/* init */
void aoa_pwm_init(void)
{
  aoa_pwm.offset = AOA_OFFSET;
  aoa_pwm.filter = AOA_FILTER;
  aoa_pwm.sens = AOA_SENS;
  aoa_pwm.angle = 0.0f;
  aoa_pwm.raw = 0.0f;
  ssa_pwm.offset = SSA_OFFSET;
  ssa_pwm.filter = SSA_FILTER;
  ssa_pwm.sens = SSA_SENS;
  ssa_pwm.angle = 0.0f;
  ssa_pwm.raw = 0.0f;
#if LOG_AOA
  log_started = false;
#endif
  aoa_send_type = SEND_TYPE_AOA;
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AOA, send_aoa);
#endif
}

/* update, log and send */
void aoa_pwm_update(void) {
#if USE_AOA || USE_SIDESLIP
  uint8_t flag = 0;
#endif
  static float prev_aoa = 0.0f;
  // raw duty cycle in usec
  uint32_t aoa_duty_raw = get_pwm_input_duty_in_usec(AOA_PWM_CHANNEL);

  // remove some offset if needed
  aoa_pwm.raw = aoa_duty_raw - AOA_PWM_OFFSET;
  // FIXME for some reason, the last value of the MA3 encoder is not 4096 but 4097
  // this case is not handled since we don't care about angles close to +- 180 deg
  aoa_pwm.angle = AOA_SIGN * (((float)aoa_pwm.raw * aoa_pwm.sens) - aoa_pwm.offset - AOA_ANGLE_OFFSET);
  // filter angle
  aoa_pwm.angle = aoa_pwm.filter * prev_aoa + (1.0f - aoa_pwm.filter) * aoa_pwm.angle;
  prev_aoa = aoa_pwm.angle;

#if USE_AOA
  SetBit(flag, 0);
#endif

#if USE_SIDESLIP
  static float prev_ssa = 0.0f;
  // raw duty cycle in usec
  uint32_t ssa_duty_raw = get_pwm_input_duty_in_usec(SSA_PWM_CHANNEL);

  // remove some offset if needed
  ssa_pwm.raw = ssa_duty_raw - AOA_PWM_OFFSET;
  // FIXME for some reason, the last value of the MA3 encoder is not 4096 but 4097
  // this case is not handled since we don't care about angles close to +- 180 deg
  ssa_pwm.angle = SSA_SIGN * (((float)ssa_pwm.raw * ssa_pwm.sens) - ssa_pwm.offset - AOA_ANGLE_OFFSET);
  // filter angle
  ssa_pwm.angle = ssa_pwm.filter * prev_ssa + (1.0f - ssa_pwm.filter) * ssa_pwm.angle;
  prev_ssa = ssa_pwm.angle;

  SetBit(flag, 1);
#endif

#if USE_AOA || USE_SIDESLIP
  AbiSendMsgINCIDENCE(AOA_PWM_ID, flag, aoa_pwm.angle, ssa_pwm.angle);
#endif

#if SEND_SYNC_AOA
  RunOnceEvery(10, send_aoa(&(DefaultChannel).trans_tx, &(DefaultDevice).device));
#endif

#if LOG_AOA
  if(pprzLogFile != -1) {
    if (!log_started) {
      sdLogWriteLog(pprzLogFile, "AOA_PWM: AOA_ANGLE(deg) AOA_RAW(int16) SSA_ANGLE(deg) SSA_RAW(int16)\n");
      log_started = true;
    } else {
      float aoa_angle = DegOfRad(aoa_pwm.angle);
      float ssa_angle = DegOfRad(ssa_pwm.angle);
      sdLogWriteLog(pprzLogFile, "AOA_PWM: %.3f %d %.3f %d\n", aoa_angle, aoa_pwm.raw, ssa_angle, ssa_pwm.raw);
    }
  }
#endif
}

