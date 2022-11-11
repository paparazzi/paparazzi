/*
 * Copyright (C) 2008-2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include <inttypes.h>

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "modules/datalink/downlink.h"
#include "modules/datalink/pprz_dl.h"
#include "modules/radio_control/radio_control.h"

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);
static        void main_on_radio_control_frame(void);

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
  radio_control_init();
  datalink_init();
  downlink_init();
  pprz_dl_init();
}

extern uint32_t debug_len;

static inline void main_periodic_task(void)
{

  RunOnceEvery(51, {
    /*LED_TOGGLE(2);*/
    uint32_t sec = sys_time.nb_sec;
    DOWNLINK_SEND_TIME(DefaultChannel, DefaultDevice, &sec);
  });

  RunOnceEvery(10, {radio_control_periodic_task();});

  int16_t foo = 0;
  RunOnceEvery(10, {
    DOWNLINK_SEND_ROTORCRAFT_RADIO_CONTROL(DefaultChannel, DefaultDevice,  \
    &radio_control.values[RADIO_ROLL], \
    &radio_control.values[RADIO_PITCH], \
    &radio_control.values[RADIO_YAW], \
    &radio_control.values[RADIO_THROTTLE], \
    &radio_control.values[RADIO_MODE], \
    &foo,        \
    &radio_control.status);
  });
#ifdef RADIO_CONTROL_TYPE_PPM
  RunOnceEvery(10,
  {uint8_t blaa = 0; DOWNLINK_SEND_PPM(DefaultChannel, DefaultDevice, &blaa, 8, ppm_pulses);});
#endif

}

static inline void main_event_task(void)
{
  mcu_event();
  //RadioControlEvent(main_on_radio_control_frame); FIXME
}

static void main_on_radio_control_frame(void)
{

  //  RunOnceEvery(10, {DOWNLINK_SEND_RC(RADIO_CONTROL_NB_CHANNEL, radio_control.values);});

}
