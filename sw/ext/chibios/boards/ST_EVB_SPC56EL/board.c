/*
    SPC5 HAL - Copyright (C) 2013 STMicroelectronics

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

#if HAL_USE_PAL || defined(__DOXYGEN__)
/* Initial setup of all defined pads, the list is terminated by a {0, 0, 0}.*/
static const spc_siu_init_t spc_siu_init[] = {
  {PCR(PORT_B, PB_LIN0_TDX), PAL_HIGH, PAL_MODE_OUTPUT_ALTERNATE(1)},
  {PCR(PORT_B, PB_LIN0_RDX), PAL_HIGH, PAL_MODE_INPUT},
  {PCR(PORT_D, PD_BUTTON1),  PAL_LOW,  PAL_MODE_INPUT},
  {PCR(PORT_D, PD_BUTTON2),  PAL_LOW,  PAL_MODE_INPUT},
  {PCR(PORT_D, PD_BUTTON3),  PAL_LOW,  PAL_MODE_INPUT},
  {PCR(PORT_D, PD_BUTTON4),  PAL_LOW,  PAL_MODE_INPUT},
  {PCR(PORT_D, PD_LED1),     PAL_HIGH, PAL_MODE_OUTPUT_PUSHPULL},
  {PCR(PORT_D, PD_LED2),     PAL_HIGH, PAL_MODE_OUTPUT_PUSHPULL},
  {PCR(PORT_D, PD_LED3),     PAL_HIGH, PAL_MODE_OUTPUT_PUSHPULL},
  {PCR(PORT_D, PD_LED4),     PAL_HIGH, PAL_MODE_OUTPUT_PUSHPULL},
  {0, 0, 0}
};

/* Initialization array for the PSMI registers.*/
static const uint8_t spc_padsels_init[SPC5_SIUL_NUM_PADSELS] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/**
 * @brief   PAL setup.
 */
const PALConfig pal_default_config = {
  PAL_MODE_UNCONNECTED,             /* Default mode for all undefined pads. */
  spc_siu_init,
  spc_padsels_init
};
#endif

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {

  spc_early_init();
}

/*
 * Board-specific initialization code.
 */
void boardInit(void) {

}
