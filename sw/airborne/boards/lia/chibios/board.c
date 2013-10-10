/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    modified by: AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
    Utah State University, http://aggieair.usu.edu/

    Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
    Calvin Coopmans (c.r.coopmans@ieee.org)

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
const PALConfig pal_default_config =
{
  {VAL_GPIOAODR, VAL_GPIOACRL, VAL_GPIOACRH},
  {VAL_GPIOBODR, VAL_GPIOBCRL, VAL_GPIOBCRH},
  {VAL_GPIOCODR, VAL_GPIOCCRL, VAL_GPIOCCRH},
  {VAL_GPIODODR, VAL_GPIODCRL, VAL_GPIODCRH},
  {VAL_GPIOEODR, VAL_GPIOECRL, VAL_GPIOECRH},
};
#endif

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {

  stm32_clock_init();
}

#if HAL_USE_MMC_SPI
/*
 * Card detection through the card internal pull-up on D3.
 */
bool_t mmc_lld_is_card_inserted(MMCDriver *mmcp) {
  static bool_t last_status = FALSE;
//  (void)mmcp;
//  if ((palReadLatch(GPIOA) & PAL_PORT_BIT(GPIOA_SPI3_CS_MMC)) == 0)
    return last_status;
//  return last_status = (bool_t)palReadPad(GPIOA, GPIOA_SPI3_CS_MMC);
}

/*
 * Card write protection detection is not possible, the card is always
 * reported as not protected.
 */
bool_t mmc_lld_is_write_protected(MMCDriver *mmcp) {

  (void)mmcp;
  return FALSE;
}
#endif

/*
 * Board-specific initialization code.
 */
void boardInit(void) {

  /*
   * Several I/O pins are re-mapped:
   *   JTAG TRST to LED
   */
  AFIO->MAPR |= AFIO_MAPR_USART3_REMAP_PARTIALREMAP | AFIO_MAPR_SWJ_CFG_NOJNTRST ;
}
