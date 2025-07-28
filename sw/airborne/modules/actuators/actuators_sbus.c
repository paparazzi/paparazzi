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
 *  Channels min, averga and maximum should be: 172, 992, 1811
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
  // Try to invert RX data logic when available in hardware periph
  uart_periph_invert_data_logic(&ACTUATORS_SBUS_DEV, true, true);
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
  uint8_t frame[25] = {0};

  frame[0] = SBUS_START_BYTE;
  frame[1] =  (actuators_sbus.cmds[0] & 0x07FF);
  frame[2] =  (actuators_sbus.cmds[0] & 0x07FF) >> 8 | (actuators_sbus.cmds[1] & 0x07FF) << 3;
  frame[3] =  (actuators_sbus.cmds[1] & 0x07FF) >> 5 | (actuators_sbus.cmds[2] & 0x07FF) << 6;
  frame[4] =  (actuators_sbus.cmds[2] & 0x07FF) >> 2;
  frame[5] =  (actuators_sbus.cmds[2] & 0x07FF) >> 10 | (actuators_sbus.cmds[3] & 0x07FF) << 1;
  frame[6] =  (actuators_sbus.cmds[3] & 0x07FF) >> 7 | (actuators_sbus.cmds[4] & 0x07FF) << 4;
  frame[7] =  (actuators_sbus.cmds[4] & 0x07FF) >> 4 | (actuators_sbus.cmds[5] & 0x07FF) << 7;
  frame[8] =  (actuators_sbus.cmds[5] & 0x07FF) >> 1;
  frame[9] =  (actuators_sbus.cmds[5] & 0x07FF) >> 9  | (actuators_sbus.cmds[6] & 0x07FF) << 2;
  frame[10] = (actuators_sbus.cmds[6] & 0x07FF) >> 6  | (actuators_sbus.cmds[7] & 0x07FF) << 5;
  frame[11] = (actuators_sbus.cmds[7] & 0x07FF) >> 3;
  frame[12] = (actuators_sbus.cmds[8] & 0x07FF);
  frame[13] = (actuators_sbus.cmds[8] & 0x07FF) >> 8 | (actuators_sbus.cmds[9]  & 0x07FF) << 3;
  frame[14] = (actuators_sbus.cmds[9] & 0x07FF) >> 5 | (actuators_sbus.cmds[10] & 0x07FF) << 6;
  frame[15] = (actuators_sbus.cmds[10] & 0x07FF) >> 2;
  frame[16] = (actuators_sbus.cmds[10] & 0x07FF) >> 10 | (actuators_sbus.cmds[11] & 0x07FF) << 1;
  frame[17] = (actuators_sbus.cmds[11] & 0x07FF) >> 7  | (actuators_sbus.cmds[12] & 0x07FF) << 4;
  frame[18] = (actuators_sbus.cmds[12] & 0x07FF) >> 4  | (actuators_sbus.cmds[13] & 0x07FF) << 7;
  frame[19] = (actuators_sbus.cmds[13] & 0x07FF) >> 1;
  frame[20] = (actuators_sbus.cmds[13] & 0x07FF) >> 9  | (actuators_sbus.cmds[14] & 0x07FF) << 2;
  frame[21] = (actuators_sbus.cmds[14] & 0x07FF) >> 6  | (actuators_sbus.cmds[15] & 0x07FF) << 5;
  frame[22] = (actuators_sbus.cmds[15] & 0x07FF) >> 3;
  frame[23] = 0x00; // No frame lost, switches off, no failsafe
  frame[24] = SBUS_END_BYTE;  // stop byte
  
  dev->put_buffer(dev->periph, 0, frame, 25);

}
