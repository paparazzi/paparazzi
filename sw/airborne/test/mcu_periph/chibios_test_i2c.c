/*
 * Copyright (C) 2010 The Paparazzi Team
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

#define DATALINK_C

/* ChibiOS includes */
#include "ch.h"

//#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/i2c.h"
#include "subsystems/datalink/downlink.h"

#define NB_ADC 8

#ifndef AD7997_I2C_DEV
#define AD7997_I2C_DEV i2c2
#endif

#define AD7997_ADDR 0x40

/* Config register addres */
#define AD7997_CONFIG_REG_ADDR 0x2

// For all channels + filter on I2C
#define AD7997_CONFIG_DATA_0 0xF // first byte
#define AD7997_CONFIG_DATA_1 0xF8// second byte

/* Initiate conversion and sequential read */
#define AD7997_READ_SEQUENTIAL 0x70

enum ad7997_stat{
  AD7997_UNINIT,
  AD7997_INIT,
  AD7997_READING
};

struct i2c_transaction ad7997_trans;
uint8_t ad7997_status = AD7997_UNINIT;

uint16_t balancer_ports[] = {0,0,0,0,0,0,0,0};

void read_ad7997_balancer(void);

/*
 * Red LEDs blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThdBlinker, 128);
static void ThdBlinker(void *arg) {

	(void) arg;
	chRegSetThreadName("blinker");
	while (TRUE) {
#ifdef SYS_TIME_LED
		LED_TOGGLE(SYS_TIME_LED);
#endif
		DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
		uint32_t sec = sys_time.nb_sec;
		DOWNLINK_SEND_TIME(DefaultChannel, DefaultDevice, &sec);

		uint8_t id = 42;
		DOWNLINK_SEND_ADC(DefaultChannel, DefaultDevice, &id, NB_ADC, balancer_ports);
		sys_time_ssleep(1);
	}
}


int main(void) {
	mcu_init();
	downlink_init();

	  //configure MODE 2 - Sequence operation
	  ad7997_trans.buf[0] = AD7997_CONFIG_REG_ADDR;
	  ad7997_trans.buf[1] = AD7997_CONFIG_DATA_0;
	  ad7997_trans.buf[2] = AD7997_CONFIG_DATA_1;
	  i2c_transmit(&AD7997_I2C_DEV, &ad7997_trans, AD7997_ADDR, 3);
	  ad7997_status = AD7997_INIT;

	/*
	 * Creates the blinker thread.
	 */
	chThdCreateStatic(waThdBlinker, sizeof(waThdBlinker), NORMALPRIO,
			ThdBlinker, NULL);

	while (TRUE) {

		read_ad7997_balancer();

		// sleep for 100ms
		sys_time_msleep(100);
	}
	return 0;
}

void read_ad7997_balancer(void) {
   if (ad7997_trans.status == I2CTransSuccess) {
        switch (ad7997_status) {
          case AD7997_INIT:
              ad7997_trans.buf[0] = AD7997_READ_SEQUENTIAL;
              i2c_transceive(&AD7997_I2C_DEV, &ad7997_trans, AD7997_ADDR, 1, 16);
              ad7997_status = AD7997_READING;
              break;
          case AD7997_READING:
              balancer_ports[0] = (ad7997_trans.buf[0] << 8 | ad7997_trans.buf[1]) & 0xFFC;
              balancer_ports[1] = (ad7997_trans.buf[2] << 8 | ad7997_trans.buf[3]) & 0xFFC;
              balancer_ports[2] = (ad7997_trans.buf[4] << 8 | ad7997_trans.buf[5]) & 0xFFC;
              balancer_ports[3] = (ad7997_trans.buf[6] << 8 | ad7997_trans.buf[7]) & 0xFFC;
              balancer_ports[4] = (ad7997_trans.buf[8] << 8 | ad7997_trans.buf[9]) & 0xFFC;
              balancer_ports[5] = (ad7997_trans.buf[10] << 8 | ad7997_trans.buf[11]) & 0xFFC;
              balancer_ports[6] = (ad7997_trans.buf[12] << 8 | ad7997_trans.buf[13]) & 0xFFC;
              balancer_ports[7] = (ad7997_trans.buf[14] << 8 | ad7997_trans.buf[15]) & 0xFFC;
              ad7997_status = AD7997_INIT;
              break;
          default:
              break;
        }
     }
  else {
          //configure MODE 2 - Sequence operation
          ad7997_trans.buf[0] = AD7997_CONFIG_REG_ADDR;
          ad7997_trans.buf[1] = AD7997_CONFIG_DATA_0;
          ad7997_trans.buf[2] = AD7997_CONFIG_DATA_1;
          i2c_transmit(&AD7997_I2C_DEV, &ad7997_trans, AD7997_ADDR, 3);
          ad7997_status = AD7997_INIT;
      }
}
