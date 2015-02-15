/*
 * Copyright (C) 2012 Piotr Esden-Tempski <piotr@esden.net>
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
 * @file test_can.c
 *
 */

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/can.h"

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);
void main_on_can_msg(uint32_t id, uint8_t *data, int len);

uint8_t tx_data[8];
uint8_t rx_data[8];
bool new_can_data = false;

int main(void)
{
  main_init();

  tx_data[0] = 0;
  tx_data[1] = 0;
  tx_data[2] = 0;
  tx_data[3] = 0;
  tx_data[4] = 0;
  tx_data[5] = 0;
  tx_data[6] = 0;
  tx_data[7] = 0;

  new_can_data = false;

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
  sys_time_register_timer((0.5 / PERIODIC_FREQUENCY), NULL);
  downlink_init();
  ppz_can_init(main_on_can_msg);
}

static inline void main_periodic_task(void)
{

  tx_data[0] += 1;
  ppz_can_transmit(0, tx_data, 8);

  LED_PERIODIC();
  DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
}



static inline void main_event_task(void)
{

  if (new_can_data) {
    if (rx_data[0] & 0x10) {
      LED_ON(2);
    } else {
      LED_OFF(2);
    }
  }

  if (new_can_data) {
    if (rx_data[0] & 0x20) {
      LED_ON(3);
    } else {
      LED_OFF(3);
    }
  }

  if (new_can_data) {
    if (rx_data[0] & 0x40) {
      LED_ON(4);
    } else {
      LED_OFF(4);
    }
  }

  if (new_can_data) {
    if (rx_data[0] & 0x80) {
      LED_ON(5);
    } else {
      LED_OFF(5);
    }
  }

}

void main_on_can_msg(uint32_t id __attribute__((unused)), uint8_t *data, int len __attribute__((unused)))
{
  for (int i = 0; i < 8; i++) {
    rx_data[i] = data[i];
  }

  new_can_data = true;
}
