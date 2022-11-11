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

/**
 * @file test_settings.c
 *
 * Test persistent settings, use together with conf/settings/test_settings.xml
 */

#include BOARD_CONFIG

#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "modules/datalink/downlink.h"
#include "modules/datalink/datalink.h"
#include "modules/datalink/pprz_dl.h"
#include "modules/core/settings.h"
#include "generated/settings.h"

#if USE_UDP
#include "mcu_periph/udp.h"
#endif

static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);


float setting_f;
uint8_t setting_u8;
double setting_d;
int32_t setting_i32;

int main(void)
{
  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic();
    }
    main_event();
  }
  return 0;
}


static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  settings_init();
  pprz_dl_init();

  datalink_init();
#if DOWNLINK
  downlink_init();
#endif
}

static inline void main_periodic(void)
{
  RunOnceEvery(100, {
    DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
    PeriodicSendDlValue(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
  });

}

static inline void main_event(void)
{
  mcu_event();
  pprz_dl_event();
}

void dl_parse_msg(struct link_device *dev __attribute__((unused)), struct transport_tx *trans __attribute__((unused)), uint8_t *buf)
{
  uint8_t msg_id = buf[1];
  switch (msg_id) {

    case  DL_PING: {
      DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
    }
    break;
    case DL_SETTING:
      if (DL_SETTING_ac_id(buf) == AC_ID) {
        uint8_t i = DL_SETTING_index(buf);
        float val = DL_SETTING_value(buf);
        DlSetting(i, val);
        DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
      }
      break;
    case DL_GET_SETTING : {
      if (DL_GET_SETTING_ac_id(buf) != AC_ID) { break; }
      uint8_t i = DL_GET_SETTING_index(buf);
      float val = settings_get_value(i);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
    }
    break;
    default:
      break;
  }
}
