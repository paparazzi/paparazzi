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
#include "test.h"

/*
 * This is a periodic thread that does absolutely nothing except flashing LEDs.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (TRUE) {
    palClearPad(GPIOC, GPIOC_LED1);
    chThdSleepMilliseconds(250);
    palSetPad(GPIOC, GPIOC_LED1);
    palClearPad(GPIOC, GPIOC_LED2);
    chThdSleepMilliseconds(250);
    palSetPad(GPIOC, GPIOC_LED2);
    palClearPad(GPIOC, GPIOC_LED3);
    chThdSleepMilliseconds(250);
    palSetPad(GPIOC, GPIOC_LED3);
    palClearPad(GPIOC, GPIOC_LED4);
    chThdSleepMilliseconds(250);
    palSetPad(GPIOC, GPIOC_LED4);
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
   * Activates the serial driver 2 using the default configuration, pins
   * are pre-configured in board.h.
   */
  sdStart(&SD2, NULL);

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state, when the button is
   * pressed the test procedure is launched.
   */
  while (TRUE) {
    if (palReadPad(GPIOA, GPIOA_WKUP_BUTTON))
      TestThread(&SD2);
    chThdSleepMilliseconds(500);
  }
}
