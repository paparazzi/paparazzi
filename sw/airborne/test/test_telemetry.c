/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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
 * @file test_telemetry.c
 *
 * Periodically sends ALIVE telemetry messages.
 */

#define DATALINK_C

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "modules/datalink/pprz_dl.h"
#include "led.h"

static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);

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
  mcu_int_enable();

  downlink_init();
  pprz_dl_init();
}

static inline void main_periodic(void)
{
  RunOnceEvery(50, {
      DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
#ifdef UART_TX_LED
      LED_TOGGLE(UART_TX_LED);
#endif
    });
  LED_PERIODIC();
}

static inline void main_event(void)
{
  mcu_event();
}

void dl_parse_msg(struct link_device *dev __attribute__((unused)),
                  struct transport_tx *trans __attribute__((unused)),
                  uint8_t *buf __attribute__((unused)))
{
}
