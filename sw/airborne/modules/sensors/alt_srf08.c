/*
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2002  Chris efstathiou hendrix@otenet.gr
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
 *
 */

/**
 * @file modules/sensors/alt_srf08.c
 * @brief Basic library for SRF08 telemeter
 *
 */

#include "mcu_periph/i2c.h"
#include "alt_srf08.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "led.h"


#ifndef SRF08_I2C_DEV
#define SRF08_I2C_DEV i2c0
#endif

/* Global Variables */
bool_t srf08_received, srf08_got;
struct i2c_transaction srf_trans;
uint16_t srf08_range;


/*###########################################################################*/

void srf08_init(void)
{
  srf08_received = FALSE;
  srf08_got = FALSE;

  srf_trans.buf[0] = 0x00;
  srf_trans.buf[1] = 0x51;
  i2c_transmit(&SRF08_I2C_DEV, &srf_trans, SRF08_UNIT_0, 2);

  /** Setting the gain to the minimun value (to avoid echos ?) */
  srf_trans.buf[0] = SRF08_SET_GAIN;
  srf_trans.buf[1] = SRF08_MIN_GAIN;
  i2c_transmit(&SRF08_I2C_DEV, &srf_trans, SRF08_UNIT_0, 2);

  return;
}
/*###########################################################################*/

void srf08_initiate_ranging(void)
{
  LED_ON(2);
  srf_trans.buf[0] = SRF08_COMMAND;
  srf_trans.buf[1] = SRF08_CENTIMETERS;
  i2c_transmit(&SRF08_I2C_DEV, &srf_trans, SRF08_UNIT_0, 2);
}

/** Ask the value to the device */
void srf08_receive(void)
{
  LED_OFF(2);
  srf_trans.buf[0] = SRF08_ECHO_1;
  srf08_received = TRUE;
  i2c_transmit(&SRF08_I2C_DEV, &srf_trans, SRF08_UNIT_0, 1);
}

/** Read values on the bus */
void srf08_read(void)
{
  srf08_got = TRUE;
  i2c_receive(&SRF08_I2C_DEV, &srf_trans, SRF08_UNIT_0, 2);
}

/** Copy the I2C buffer */
void srf08_copy(void)
{
  srf08_range = srf_trans.buf[0] << 8 | srf_trans.buf[1];
}

void srf08_ping()
{
  srf08_initiate_ranging();
  while (srf_trans.status != I2CTransSuccess);  /* blocking */

  srf08_receive();
}
/*###########################################################################*/

uint32_t srf08_read_register(uint8_t srf08_register)
{
  uint8_t cnt;

  union i2c_union {
    uint32_t  rx_word;
    uint8_t   rx_byte[2];
  } i2c;


  srf_trans.buf[0] = srf08_register;

  /* get high byte msb first */
  if (srf08_register >= 2) {
    cnt = 2;
  } else {
    cnt = 1;
  }

  i2c_transceive(&SRF08_I2C_DEV, &srf_trans, SRF08_UNIT_0, 1, cnt);

  /* get high byte msb first */
  if (srf08_register >= 2) {
    i2c.rx_byte[1] = srf_trans.buf[1];
  }

  /* get low byte msb first  */
  i2c.rx_byte[0] = srf_trans.buf[0];

  return (i2c.rx_word);
}

void srf08_event(void)
{
  float f = 0;
  uint8_t i = 0;

  /** Handling of data sent by the device (initiated by srf08_receive() */
  if (srf_trans.status == I2CTransSuccess) {
    if (srf08_received) {
      srf08_received = FALSE;
      srf08_read();
    } else if (srf08_got) {
      srf08_got = FALSE;
      srf08_copy();
      DOWNLINK_SEND_RANGEFINDER(DefaultChannel, DefaultDevice, &srf08_range, &f, &f, &f, &f, &f, &i);
    }
  }
}
