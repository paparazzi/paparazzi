/*
 * Copyright (C) 2010 Martin Mueller
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

/** \file humid_sht_i2c.c
 *  \brief Sensirion SHT25 humidity/temperature sensor interface
 *
 */


#include "modules/meteo/humid_sht_i2c.h"

#include "mcu_periph/i2c.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"


#ifndef SHT_I2C_DEV
#define SHT_I2C_DEV i2c0
#endif

#define SHT_SLAVE_ADDR 0x80

struct i2c_transaction sht_trans;
uint8_t sht_status;
uint8_t sht_serial[8] = {0};
uint32_t sht_serial1 = 0, sht_serial2 = 0;
uint16_t humidsht_i2c, tempsht_i2c;
float fhumidsht_i2c, ftempsht_i2c;

int8_t humid_sht_crc(volatile uint8_t *data)
{
  uint8_t i, bit, crc = 0;

  for (i = 0; i < 2; i++) {
    crc ^= (data[i]);
    for (bit = 8; bit > 0; bit--) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x131;
      } else {
        crc = (crc << 1);
      }
    }
  }
  if (crc != data[2]) {
    return -1;
  } else {
    return 0;
  }
}

void humid_sht_init_i2c(void)
{
  sht_status = SHT2_UNINIT;
}

void humid_sht_periodic_i2c(void)
{
  switch (sht_status) {

    case SHT2_UNINIT:
      /* do soft reset, then wait at least 15ms */
      sht_status = SHT2_RESET;
      sht_trans.buf[0] = SHT2_SOFT_RESET;
      i2c_transmit(&SHT_I2C_DEV, &sht_trans, SHT_SLAVE_ADDR, 1);
      break;

    case SHT2_SERIAL:
      /* get serial number part 1 */
      sht_status = SHT2_SERIAL1;
      sht_trans.buf[0] = 0xFA;
      sht_trans.buf[1] = 0x0F;
      i2c_transceive(&SHT_I2C_DEV, &sht_trans, SHT_SLAVE_ADDR, 2, 8);
      break;

    case SHT2_SERIAL1:
    case SHT2_SERIAL2:
      break;

    default:
      /* trigger temp measurement, no master hold */
      sht_trans.buf[0] = SHT2_TRIGGER_TEMP;
      sht_status = SHT2_TRIG_TEMP;
      i2c_transmit(&SHT_I2C_DEV, &sht_trans, SHT_SLAVE_ADDR, 1);
      /* send serial number every 30 seconds */
      RunOnceEvery((4 * 30), DOWNLINK_SEND_SHT_I2C_SERIAL(DefaultChannel, DefaultDevice, &sht_serial1, &sht_serial2));
      break;
  }
}

/* needs 85ms delay from temp trigger measurement */
void humid_sht_p_temp(void)
{
  if (sht_status == SHT2_GET_TEMP) {
    /* get temp */
    sht_status = SHT2_READ_TEMP;
    i2c_receive(&SHT_I2C_DEV, &sht_trans, SHT_SLAVE_ADDR, 3);
  }
}

/* needs 29ms delay from humid trigger measurement */
void humid_sht_p_humid(void)
{
  if (sht_status == SHT2_GET_HUMID) {
    /* read humid */
    sht_status = SHT2_READ_HUMID;
    i2c_receive(&SHT_I2C_DEV, &sht_trans, SHT_SLAVE_ADDR, 3);
  }
}

void humid_sht_event_i2c(void)
{
  if (sht_trans.status == I2CTransSuccess) {
    switch (sht_status) {

      case SHT2_TRIG_TEMP:
        sht_status = SHT2_GET_TEMP;
        sht_trans.status = I2CTransDone;
        break;

      case SHT2_READ_TEMP:
        /* read temperature */
        tempsht_i2c = (sht_trans.buf[0] << 8) | sht_trans.buf[1];
        tempsht_i2c &= 0xFFFC;
        if (humid_sht_crc(sht_trans.buf) == 0) {
          /* trigger humid measurement, no master hold */
          sht_trans.buf[0] = SHT2_TRIGGER_HUMID;
          sht_status = SHT2_TRIG_HUMID;
          i2c_transmit(&SHT_I2C_DEV, &sht_trans, SHT_SLAVE_ADDR, 1);
        } else {
          /* checksum error, restart */
          sht_status = SHT2_IDLE;
          sht_trans.status = I2CTransDone;
        }
        break;

      case SHT2_TRIG_HUMID:
        sht_status = SHT2_GET_HUMID;
        sht_trans.status = I2CTransDone;
        break;

      case SHT2_READ_HUMID:
        /* read humidity */
        humidsht_i2c = (sht_trans.buf[0] << 8) | sht_trans.buf[1];
        humidsht_i2c &= 0xFFFC;
        fhumidsht_i2c = -6. + 125. / 65536. * humidsht_i2c;
        ftempsht_i2c = -46.85 + 175.72 / 65536. * tempsht_i2c;

        sht_status = SHT2_IDLE;
        sht_trans.status = I2CTransDone;

        if (humid_sht_crc(sht_trans.buf) == 0) {
          DOWNLINK_SEND_SHT_I2C_STATUS(DefaultChannel, DefaultDevice, &humidsht_i2c, &tempsht_i2c, &fhumidsht_i2c, &ftempsht_i2c);
        }
        break;

      case SHT2_RESET:
        sht_status = SHT2_SERIAL;
        sht_trans.status = I2CTransDone;
        break;

      case SHT2_SERIAL1:
        /* read serial number part 1 */
        sht_serial[5] = sht_trans.buf[0];
        sht_serial[4] = sht_trans.buf[2];
        sht_serial[3] = sht_trans.buf[4];
        sht_serial[2] = sht_trans.buf[6];
        /* get serial number part 2 */
        sht_status = SHT2_SERIAL2;
        sht_trans.buf[0] = 0xFC;
        sht_trans.buf[1] = 0xC9;
        i2c_transceive(&SHT_I2C_DEV, &sht_trans, SHT_SLAVE_ADDR, 2, 6);
        break;

      case SHT2_SERIAL2:
        /* read serial number part 2 */
        sht_serial[1] = sht_trans.buf[0];
        sht_serial[0] = sht_trans.buf[1];
        sht_serial[7] = sht_trans.buf[3];
        sht_serial[6] = sht_trans.buf[4];
        sht_serial1 = sht_serial[7] << 24 | sht_serial[6] << 16 | sht_serial[5] << 8 | sht_serial[4];
        sht_serial2 = sht_serial[3] << 24 | sht_serial[2] << 16 | sht_serial[1] << 8 | sht_serial[0];
        DOWNLINK_SEND_SHT_I2C_SERIAL(DefaultChannel, DefaultDevice, &sht_serial1, &sht_serial2);
        sht_status = SHT2_IDLE;
        sht_trans.status = I2CTransDone;
        break;

      default:
        sht_trans.status = I2CTransDone;
        break;
    }
  }
}

