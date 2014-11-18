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
 */

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

/* UART1 */
#define A_PORT     GPIOA
#define A_RX_PIN   GPIO10
#define A_RX_PORT  A_PORT
#define A_TX_PIN   GPIO9
#define A_TX_PORT  A_PORT

/* UART2 */
#define B_PORT     GPIOA
#define B_RX_PIN   GPIO3
#define B_RX_PORT  B_PORT
#define B_TX_PIN   GPIO2
#define B_TX_PORT  B_PORT

static inline void main_periodic( void );
static inline void main_event( void );
void Delay(volatile uint32_t nCount);

void Delay(volatile uint32_t nCount) {
  for(; nCount != 0; nCount--);
}

int main(void) {

  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);

  /* init RCC */
  rcc_periph_clock_enable(RCC_GPIOA);

  /* Init GPIO for rx pins */
  gpio_set(A_RX_PORT, A_RX_PIN);
  gpio_set_mode(A_RX_PORT, GPIO_MODE_INPUT,
          GPIO_CNF_INPUT_PULL_UPDOWN, A_RX_PIN);
  gpio_set(B_RX_PORT, B_RX_PIN);
  gpio_set_mode(B_RX_PORT, GPIO_MODE_INPUT,
          GPIO_CNF_INPUT_PULL_UPDOWN, B_RX_PIN);

  /* Init GPIO for tx pins */
  gpio_set_mode(A_RX_PORT, GPIO_MODE_OUTPUT_50_MHZ,
          GPIO_CNF_OUTPUT_PUSHPULL, A_TX_PIN);
  gpio_set_mode(B_RX_PORT, GPIO_MODE_OUTPUT_50_MHZ,
          GPIO_CNF_OUTPUT_PUSHPULL, B_TX_PIN);

  gpio_clear(A_TX_PORT, A_TX_PIN);

  /* */
  while (1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic();
    main_event();
  }

  return 0;
}



static inline void main_periodic( void ) {
  LED_PERIODIC();
}

static inline void main_event( void ) {
  //  Delay(2000);
  static uint8_t foo = 0;
  foo++;

#if 0
  if (!(foo%2))
    gpio_set(B_TX_PORT, B_TX_PIN);
  else
    gpio_clear(B_TX_PORT, B_TX_PIN);
#endif

#if 0
  if (!(foo%2))
    gpio_clear(A_TX_PORT, A_TX_PIN);
  else
    gpio_set(A_TX_PORT, A_TX_PIN);
#endif

#if 1
  /* passthrough B_RX to A_TX */
  if (GPIO_IDR(B_RX_PORT) & B_RX_PIN)
    gpio_set(A_TX_PORT, A_TX_PIN);
  else
    gpio_clear(A_TX_PORT, A_TX_PIN);
#endif
  /* passthrough A_RX to B_TX */
  if (gpio_get(A_RX_PORT, A_RX_PIN)) {
    gpio_set(B_TX_PORT, B_TX_PIN);
    LED_ON(2);
  }
  else {
    gpio_clear(B_TX_PORT, B_TX_PIN);
    LED_OFF(2);
  }


}
