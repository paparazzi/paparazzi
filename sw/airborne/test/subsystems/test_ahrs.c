/*
 * Copyright (C) 2011 The Paparazzi Team
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

#include <inttypes.h>

#define DATALINK_C

/* PERIODIC_C_MAIN is defined before generated/periodic_telemetry.h
 * in order to implement telemetry_mode_Main_*
 */
#define PERIODIC_C_MAIN

#include "generated/periodic_telemetry.h"

#define ABI_C
#include "subsystems/abi.h"

#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/i2c.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/telemetry.h"

#include "subsystems/datalink/datalink.h"
#include "generated/settings.h"

#include "subsystems/imu.h"
#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_aligner.h"

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);
static inline void main_report(void);

int main(void)
{
  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
    main_event_task();
  }
  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  imu_init();
#if USE_AHRS_ALIGNER
  ahrs_aligner_init();
#endif
  ahrs_init();
  downlink_init();

  mcu_int_enable();
}

static inline void main_periodic_task(void)
{
  if (sys_time.nb_sec > 1) {
    imu_periodic();
  }
  RunOnceEvery(10, { LED_PERIODIC();});
  RunOnceEvery(PERIODIC_FREQUENCY, { datalink_time++; });
  main_report();
}

static inline void main_event_task(void)
{
  mcu_event();
  DatalinkEvent();
  ImuEvent();
}

static inline void main_report(void)
{
  RunOnceEvery(512, DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM));

  periodic_telemetry_send_Main(DefaultPeriodic, &(DefaultChannel).trans_tx, &(DefaultDevice).device);
}

void dl_parse_msg(void)
{
  uint8_t msg_id = dl_buffer[1];
  switch (msg_id) {

    case  DL_PING: {
      DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
    }
    break;
    case DL_SETTING:
      if (DL_SETTING_ac_id(dl_buffer) == AC_ID) {
        uint8_t i = DL_SETTING_index(dl_buffer);
        float val = DL_SETTING_value(dl_buffer);
        DlSetting(i, val);
        DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
      }
      break;
    case DL_GET_SETTING : {
      if (DL_GET_SETTING_ac_id(dl_buffer) != AC_ID) { break; }
      uint8_t i = DL_GET_SETTING_index(dl_buffer);
      float val = settings_get_value(i);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
    }
    break;
    default:
      break;
  }
}
