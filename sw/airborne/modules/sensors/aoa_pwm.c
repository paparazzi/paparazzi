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
* SENSOR, exemple : US DIGITAL MA3-P12-125-B
* @see http://www.usdigital.com/products/encoders/absolute/rotary/shaft/ma3
*/

#include "modules/sensors/aoa_pwm.h"
#include "mcu_periph/pwm_input.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "generated/airframe.h"

#if LOG_AOA
#include "sdLog.h"
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
bool log_started;
#endif

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

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_aoa(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AOA(trans, dev, AC_ID, &aoa_pwm.raw, &aoa_pwm.angle);
}

#endif

void aoa_pwm_init(void)
{
  aoa_pwm.offset = AOA_OFFSET;
  aoa_pwm.filter = AOA_FILTER;
  aoa_pwm.sens = AOA_SENS;
  aoa_pwm.angle = 0.0f;
  aoa_pwm.raw = 0.0f;
#if LOG_AOA
  log_started = false;
#endif
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AOA, send_aoa);
#endif
}

void aoa_pwm_update(void) {
  static float prev_aoa = 0.0f;

  // raw duty cycle in usec
  uint32_t duty_raw = get_pwm_input_duty_in_usec(AOA_PWM_CHANNEL);

  // remove some offset if needed
  aoa_pwm.raw = duty_raw - AOA_PWM_OFFSET;
  // FIXME for some reason, the last value of the MA3 encoder is not 4096 but 4097
  // this case is not handled since we don't care about angles close to +- 180 deg
  aoa_pwm.angle = AOA_SIGN * (((float)aoa_pwm.raw * aoa_pwm.sens) - aoa_pwm.offset - AOA_ANGLE_OFFSET);
  // filter angle
  aoa_pwm.angle = aoa_pwm.filter * prev_aoa + (1.0f - aoa_pwm.filter) * aoa_pwm.angle;
  prev_aoa = aoa_pwm.angle;

#if USE_AOA
  stateSetAngleOfAttack_f(aoa_adc.angle);
#endif

#if SEND_SYNC_AOA
  RunOnceEvery(10, DOWNLINK_SEND_AOA(DefaultChannel, DefaultDevice, &aoa_pwm.raw, &aoa_pwm.angle));
#endif

#if LOG_AOA
  if(pprzLogFile != -1) {
    if (!log_started) {
      sdLogWriteLog(pprzLogFile, "AOA_PWM: ANGLE(deg) RAW(int16)\n");
      log_started = true;
    } else {
      float angle = DegOfRad(aoa_pwm.angle);
      sdLogWriteLog(pprzLogFile, "AOA_PWM: %.3f %d\n", angle, aoa_pwm.raw);
    }
  }
#endif
}

