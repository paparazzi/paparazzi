/*
 * $Id$
 *
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

#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "peripherals/sc18is600.h"
#include "led.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_spi2_init(void);

int main(void) {
  main_init();

  while(1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic_task();
    main_event_task();
  }

  return 0;
}


static inline void main_init( void ) {
  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  sc18is600_init();
  main_spi2_init();
}

static inline void main_periodic_task( void ) {
  //  LED_TOGGLE(6);
  RunOnceEvery(10,
	       {
		 DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
		 LED_PERIODIC();
	       });

  static uint8_t foo=0;
  switch (foo) {
  case 2:
    /* set I2C clock to 400khz */
    sc18is600_write_to_register(Sc18Is600_I2CClock, 0x05);
    break;
  case 3:
    /* read it back, just to check */
    sc18is600_read_from_register(Sc18Is600_I2CClock);
    break;
  case 4:
    /* set gyro range to 2000deg/s : write 0x03 to register at addr 0x16 */
    sc18is600.priv_tx_buf[3] = 0x16;
    sc18is600.priv_tx_buf[4] = 0x03;
    sc18is600_transmit(0xD0, 2);
    break;
  case 5:
    /* reads 8 bytes from address 0x1b */
    sc18is600.priv_tx_buf[4] = 0x1b;
    sc18is600_tranceive(0xD0, 1, 8);
    break;
  }

  if (foo< 5) foo++;

}

static inline void main_event_task( void ) {

}


static inline void main_spi2_init( void ) {

  /* set mag ss as output and assert it (on PC12) ------------------------------*/
  /* set mag reset as output and assert it (on PC13) ------------------------------*/
  /* set eeprom ss as output and assert it (on PC14) ------------------------------*/
  GPIOC->BSRR = GPIO_Pin_12;
  GPIOC->BSRR = GPIO_Pin_13;
  GPIOC->BSRR = GPIO_Pin_14;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

}

