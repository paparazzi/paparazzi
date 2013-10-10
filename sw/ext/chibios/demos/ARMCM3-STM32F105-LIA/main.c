/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "serial.h"
//#include "test.h"

/*
 * Green LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (TRUE) {
	palClearPad(GPIOC, GPIOC_LED5);
    chThdSleepMilliseconds(50);
    palSetPad(GPIOC, GPIOC_LED5);
    chThdSleepMicroseconds(50);
    palClearPad(GPIOC, GPIOC_LED4);
    chThdSleepMilliseconds(50);
    palSetPad(GPIOC, GPIOC_LED4);
    chThdSleepMicroseconds(50);
    palClearPad(GPIOC, GPIOC_LED3);
    chThdSleepMilliseconds(50);
    palSetPad(GPIOC, GPIOC_LED3);
    chThdSleepMicroseconds(50);

    palClearPad(GPIOB, GPIOB_LED2);
    chThdSleepMilliseconds(50);
    palSetPad(GPIOB, GPIOB_LED2);
    chThdSleepMicroseconds(50);

    palClearPad(GPIOA, GPIOA_LED1);
    chThdSleepMilliseconds(50);
    palSetPad(GPIOA, GPIOA_LED1);
    chThdSleepMicroseconds(50);
  }
}


/*
 * Application entry point.
 */
int main(void) {


  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 3 using the driver default configuration.
   */
  static const SerialConfig default_config =
  {
	2250000,                                   		  		  /*     BITRATE    */
    0,                                                        /*    USART CR1   */
    USART_CR2_STOP1_BITS | USART_CR2_LINEN,                   /*    USART CR2   */
    0                                                         /*    USART CR3   */
  };

  sdStart(&SD4, &default_config);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (TRUE) {
    //if (palReadPad(GPIOC, GPIOC_SWITCH_TAMPER) == 0)
    //  TestThread(&SD3);
	  uint8_t rx_buf[10] = { 0x01, 0x08, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, '\r' };
	  size_t length = sizeof(rx_buf);
	  sdWrite(&SD4, rx_buf, length);
      chThdSleepMilliseconds(500);
  }
}
