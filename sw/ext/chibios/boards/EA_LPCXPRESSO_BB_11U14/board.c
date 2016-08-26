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

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)
const PALConfig pal_default_config = {
  {VAL_GPIO0DATA, VAL_GPIO0DIR},
  {VAL_GPIO1DATA, VAL_GPIO1DIR}
};
#endif

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {

  lpc_clock_init();
}

/*
 * Board-specific initialization code.
 */
void boardInit(void) {

  /*
   * Extra, board-specific, initializations.
   * NOTE: PIO1_2 is associated also to the JTAG, if you need to use JTAG
   *       you must comment that line first.
   */
  LPC_IOCON->PIO0_7 = 0x80;             /* Disables pull-up on LED2 output. */
  LPC_IOCON->TRST_PIO0_14 = 0x81;       /* Disables pull-up on LED3B output
                                           and makes it GPIO1_2.            */
  LPC_IOCON->PIO0_21 = 0x80;            /* Disables pull-up on LED3R output.*/
  LPC_IOCON->PIO0_22 = 0x80;            /* Disables pull-up on LED3G output.*/

  /* SSP0 mapping.*/
  LPC_IOCON->PIO1_29 = 0x81;            /* SCK0 without resistors.          */
  LPC_IOCON->PIO0_8  = 0x81;            /* MISO0 without resistors.         */
  LPC_IOCON->PIO0_9  = 0x81;            /* MOSI0 without resistors.         */

  /* USART mapping.*/
  LPC_IOCON->PIO0_18 = 0x81;            /* RDX without resistors.           */
  LPC_IOCON->PIO0_19 = 0x81;            /* TDX without resistors.           */
}
