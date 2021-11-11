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

/** @file actuators_sbus.c
 *  Sbus actuator driver, which can output as 7 sbus channels at ~11ms.
 *  Channels min, averga and maximum should be: 340, 1024, 1708
 */

#include "modules/actuators/actuators.h"
#include "modules/actuators/actuators_sbus.h"
#include "generated/airframe.h"
#include "mcu_periph/uart.h"

/* Currently we only support 7 channels */
#if SERVOS_SBUS_NB > ACTUATORS_SBUS_MAX_NB
#error SBUS actuators only support less then 7 servos
#endif

/* Calculate the frequency divider to aim at 7 ms */
#if PERIODIC_FREQUENCY < 150
#error Sbus actuators need at leest a frequency of 150 Hz
#else
static uint8_t freq_trig = PERIODIC_FREQUENCY / 142.0 + 0.5; // Round it to nearest value
#endif

/* Main actuator structure */
struct ActuatorsSbus actuators_sbus;
static inline void actuators_sbus_send(struct link_device *dev);

/*
 * Initialize the sbus devices (UART output devices)
 */
void actuators_sbus_init(void)
{
  actuators_sbus.device = &((ACTUATORS_SBUS_DEV).device);

  uart_periph_set_bits_stop_parity(&ACTUATORS_SBUS_DEV, UBITS_8, USTOP_2, UPARITY_EVEN);
  uart_periph_set_baudrate(&ACTUATORS_SBUS_DEV, B100000);
}

/*
 * Transmit the sbus output at 1 / 7ms
 */
void actuators_sbus_set(void)
{
  static uint8_t cnt = 0;

  // Only send every 7 ms
  cnt++;
  if (cnt == freq_trig) {
    actuators_sbus_send(actuators_sbus.device);
    cnt = 0;
  }
}

/*
 * Actually transmit the sbus output on a link device
 *
 * The protocol is 25 Byte long and is send every 14ms (analog mode) or 7ms (highspeed mode).
 * One Byte = 1 startbit + 8 databit + 1 paritybit + 2 stopbit (8E2), baudrate = 100'000 bit/s
 * The highest bit is send first. The logic is inverted (Level High = 1)
 *
 * [startbyte] [data1] [data2] .... [data22] [flags][endbyte]
 *
 * SBUS protocol
 */

#define SBUS_START_BYTE 0x0f
#define SBUS_END_BYTE 0x00

#define SBUS_BIT_PER_CHANNEL 11

static inline void actuators_sbus_send(struct link_device *dev)
{
  uint8_t i = 0;
  uint8_t bits_sent = 0;
  uint8_t data[22];

  /* start */
  dev->put_byte(dev->periph, 0, SBUS_START_BYTE);

  /* Fill all channels */
  for (i = 0; i < 22; i++) {
    data[i] = 0x00;
  }
  for (i = 0; i < ACTUATORS_SBUS_MAX_NB; i++) {
    uint16_t chn = actuators_sbus.cmds[i] & 0x07ff; // 11 bit
    uint8_t ind = bits_sent / SBUS_BIT_PER_CHANNEL;
    uint8_t shift = bits_sent % SBUS_BIT_PER_CHANNEL;
    data[ind] |= (chn >> (3 + shift)) & 0xff; // Sends (8 - shift) bits of the 11: 11-(8-shift) remain = 3 + shift
    if (shift > 5) { // need 3 bytes to fit the 11 bits
      data[ind + 1] |= (chn >> (shift - 5)) & 0xff; // Sends next 8
      data[ind + 2] |= (chn << (3 - shift)) &
                       0xff; // Sends remaining 3 + shift - 8 bits = shift-5 bits: left aligned: 8 - (shift-5) = 3-shift
    } else { // (shift <= 5) then it fits in 2 bytes
      data[ind + 1] |= (chn << (5 - shift)) &
      0xff; // Sends remaining 3 + shift bits left aligned: 8 - (3 + shift) = 5 - shift
    }
    bits_sent += SBUS_BIT_PER_CHANNEL;
  }

  /* Transmit all channels */
  for (i = 0; i < 22; i++) {
    dev->put_byte(dev->periph, 0, data[i]);
  }

  /* flags */
  dev->put_byte(dev->periph, 0, 0x00); // No frame lost, switches off, no failsafe

  /* stop byte */
  dev->put_byte(dev->periph, 0, SBUS_END_BYTE);

}
