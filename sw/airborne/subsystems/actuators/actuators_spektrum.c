/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file actuators_spektrum.c
 *  Spektrum actuator driver, which can output as 7 spektrum channels at ~11ms.
 *  Channels min, averga and maximum should be: 340, 1024, 1708
 */

#include "subsystems/actuators.h"
#include "subsystems/actuators/actuators_spektrum.h"
#include "generated/airframe.h"
#include "mcu_periph/uart.h"

/* Currently we only support 7 channels, but could be extended to 14+ */
#if SERVOS_SPEKTRUM_NB > ACTUATORS_SPEKTRUM_MAX_NB
#error Spektrum actuators only support less then 7 servos
#endif

/* Calculate the frequency divider to aim at 11ms (~90Hz) */
#if PERIODIC_FREQUENCY < 90
#error Spektrum actuators need at leest a frequency of Hz
#else
static uint8_t freq_trig = PERIODIC_FREQUENCY / 90.0 + 0.5; // Round it to nearest value
#endif

/* Main actuator structure */
struct ActuatorsSpektrum actuators_spektrum;
static inline void actuators_spektrum_send(struct link_device *dev);

/*
 * Initialize the spektrum devices (UART output devices)
 */
void actuators_spektrum_init(void)
{
  actuators_spektrum.device = &((ACTUATORS_SPEKTRUM_DEV).device);

#ifdef ACTUATORS_SPEKTRUM_DEV2
  actuators_spektrum.device2 = &((ACTUATORS_SPEKTRUM_DEV2).device);
#endif
}

/*
 * Transmit the spektrum output at ~90Hz
 */
void actuators_spektrum_set(void)
{
  static uint8_t cnt = 0;

  // Only send every 11 ms
  cnt++;
  if (cnt == freq_trig) {
    actuators_spektrum_send(actuators_spektrum.device);

#ifdef ACTUATORS_SPEKTRUM_DEV2
    actuators_spektrum_send(actuators_spektrum.device2);
#endif

    cnt = 0;
  }
}

/*
 * Actually transmit the spektrum output on a link device
 */
static inline void actuators_spektrum_send(struct link_device *dev)
{
  uint8_t i = 0;
  dev->put_byte(dev->periph, 0x00); // number missed frames
  dev->put_byte(dev->periph, 0x12); // 7 channels, 11 bit, 11ms

  /* Transmit all channels */
  for (i = 0; i < ACTUATORS_SPEKTRUM_MAX_NB; i++) {
    dev->put_byte(dev->periph, i << 3 | actuators_spektrum.cmds[i] >> 8);
    dev->put_byte(dev->periph, actuators_spektrum.cmds[i] & 0xFF);
  }
}
