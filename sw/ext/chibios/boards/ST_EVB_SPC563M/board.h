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

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for a generic SPC563Mxx proto board.
 */

/*
 * Board identifiers.
 */
#define BOARD_GENERIC_SPC563M
#define BOARD_NAME                  "Generic SPC563Mxx"

/*
 * Board frequencies.
 */
#if !defined(SPC5_XOSC_CLK)
#define SPC5_XOSC_CLK               8000000
#endif

/*
 * I/O definitions.
 */
#define P5_ESCI_A_TX                9
#define P5_ESCI_A_RX                10

#define P11_BUTTON1                 3
#define P11_BUTTON2                 5
#define P11_BUTTON3                 7
#define P11_BUTTON4                 9

#define P11_LED1                    12
#define P11_LED2                    13
#define P11_LED3                    14
#define P11_LED4                    15

/*
 * Support macros.
 */
#define PCR(port, pin)  (((port) * 16) + (pin))

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
