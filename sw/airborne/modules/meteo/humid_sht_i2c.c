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
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"


#ifndef SHT_I2C_DEV
#define SHT_I2C_DEV i2c0
#endif

#define SHT_SLAVE_ADDR 0x80

#define SHT2_WRITE_USER          0xE6
#define SHT2_READ_USER           0xE7
#define SHT2_TRIGGER_TEMP        0xF3
#define SHT2_TRIGGER_HUMID       0xF5
#define SHT2_SOFT_RESET          0xFE

static void humid_sht_thd(void* arg);
static void sht_read_serial(struct sht_humid_t* sht);
static int sht_read_temp(struct sht_humid_t* sht);
static int sht_read_humid(struct sht_humid_t* sht);
static int8_t humid_sht_crc(volatile uint8_t *data);



static struct sht_humid_t sht;


void humid_sht_init_i2c(void)
{
  sht.sht_status = SHT2_UNINIT;
  pprz_bsem_init(&sht.bsem_sht_status, true);
  pprz_thread_create(&sht.thd_handle, 512, "humid_sht25", PPRZ_NORMAL_PRIO+1, humid_sht_thd, &sht);
}

void humid_sht_periodic_i2c(void)
{
  if(sht.sht_status == SHT2_READING) {
    /* send serial number every 30 seconds */
      RunOnceEvery((4 * 30), DOWNLINK_SEND_SHT_I2C_SERIAL(DefaultChannel, DefaultDevice, (uint32_t*)&sht.sht_serial1, (uint32_t*)&sht.sht_serial2));
  }

  if(pprz_bsem_wait_timeout(&sht.bsem_sht_status, 0) == 0) {
    DOWNLINK_SEND_SHT_I2C_STATUS(DefaultChannel, DefaultDevice, (uint16_t*)&sht.humidsht_i2c, (uint16_t*)&sht.tempsht_i2c, (float*)&sht.fhumidsht_i2c, (float*)&sht.ftempsht_i2c);
  }
}


static void humid_sht_thd(void* arg) {
  pprz_sleep_ms(100);
  struct sht_humid_t* sht = (struct sht_humid_t*)arg;

  /* soft reset sensor */
  sht->sht_trans.buf[0] = SHT2_SOFT_RESET;
  while(i2c_blocking_transmit(&SHT_I2C_DEV, &sht->sht_trans, SHT_SLAVE_ADDR, 1, 0.5) != I2CTransSuccess) {
    pprz_sleep_ms(100);
  }

  /* read serial number */
  sht_read_serial(sht);
  
  sht->sht_status = SHT2_READING;

  while(true) {
    /* read temperature */
    if(sht_read_temp(sht)) {continue;}
    /* read humidity */
    if(sht_read_humid(sht)) {continue;}
    /* signal semaphore to handle data */
    pprz_bsem_signal(&sht->bsem_sht_status);

    pprz_sleep_ms(500);
  }
}


static void sht_read_serial(struct sht_humid_t* sht) {
  uint8_t sht_serial[8] = {0};

  /* request serial number part 1 */
  sht->sht_trans.buf[0] = 0xFA;
  sht->sht_trans.buf[1] = 0x0F;
  while(i2c_blocking_transceive(&SHT_I2C_DEV, &sht->sht_trans, SHT_SLAVE_ADDR, 2, 8, 0.5) != I2CTransSuccess) {
    pprz_sleep_ms(10);
  }
  /* read serial number part 1 */
  sht_serial[5] = sht->sht_trans.buf[0];
  sht_serial[4] = sht->sht_trans.buf[2];
  sht_serial[3] = sht->sht_trans.buf[4];
  sht_serial[2] = sht->sht_trans.buf[6];
  
  /* request serial number part 2 */
  sht->sht_trans.buf[0] = 0xFC;
  sht->sht_trans.buf[1] = 0xC9;
  while(i2c_blocking_transceive(&SHT_I2C_DEV, &sht->sht_trans, SHT_SLAVE_ADDR, 2, 6, 0.5) != I2CTransSuccess) {
    pprz_sleep_ms(10);
  }
  /* read serial number part 2 */
  sht_serial[1] = sht->sht_trans.buf[0];
  sht_serial[0] = sht->sht_trans.buf[1];
  sht_serial[7] = sht->sht_trans.buf[3];
  sht_serial[6] = sht->sht_trans.buf[4];

  sht->sht_serial1 = sht_serial[7] << 24 | sht_serial[6] << 16 | sht_serial[5] << 8 | sht_serial[4];
  sht->sht_serial2 = sht_serial[3] << 24 | sht_serial[2] << 16 | sht_serial[1] << 8 | sht_serial[0];
}

static int sht_read_temp(struct sht_humid_t* sht) {
  /* trigger temp measurement, no master hold */
  sht->sht_trans.buf[0] = SHT2_TRIGGER_TEMP;
  if(i2c_blocking_transmit(&SHT_I2C_DEV, &sht->sht_trans, SHT_SLAVE_ADDR, 1, 0.5) != I2CTransSuccess ) {
    return -1;
  }
  /* needs 85ms delay from temp trigger measurement */
  pprz_sleep_ms(85);
  /* read temperature */
  if(i2c_blocking_receive(&SHT_I2C_DEV, &sht->sht_trans, SHT_SLAVE_ADDR, 3, 0.5) != I2CTransSuccess) {
    return -1;
  }
  if (humid_sht_crc(sht->sht_trans.buf) != 0) {
    /* checksum error*/
    return -1;
  }

  sht->tempsht_i2c = ((sht->sht_trans.buf[0] << 8) | sht->sht_trans.buf[1]) & 0xFFFC;
  sht->ftempsht_i2c = -46.85 + 175.72 / 65536. * sht->tempsht_i2c;
  return 0;
}

static int sht_read_humid(struct sht_humid_t* sht) {
  /* trigger humid measurement, no master hold */
  sht->sht_trans.buf[0] = SHT2_TRIGGER_HUMID;
  if(i2c_blocking_transmit(&SHT_I2C_DEV, &sht->sht_trans, SHT_SLAVE_ADDR, 1, 0.5) != I2CTransSuccess) {
    return -1;
  }
  /* needs 29ms delay from humid trigger measurement */
  pprz_sleep_ms(29);
  /* read humidity */
  if(i2c_blocking_receive(&SHT_I2C_DEV, &sht->sht_trans, SHT_SLAVE_ADDR, 3, 0.5) != I2CTransSuccess) {
    return -1;
  }
  if (humid_sht_crc(sht->sht_trans.buf) != 0) {
    /* checksum error */
    return -1;
  }
  sht->humidsht_i2c = ((sht->sht_trans.buf[0] << 8) | sht->sht_trans.buf[1]) & 0xFFFC;
  sht->fhumidsht_i2c = -6. + 125. / 65536. * sht->humidsht_i2c;
  return 0;
}

static int8_t humid_sht_crc(volatile uint8_t *data)
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
