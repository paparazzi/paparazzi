/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for Olimex LPC-P1227 board.
 *
 */

/*
 * Board identifiers.
 */
#define OLIMEX_LPC_P1227
#define BOARD_NAME "Olimex LPC-P1227"

/*
 * Board frequencies.
 */
#define SYSOSCCLK               12000000


/*
 * GPIO 0 initial setup.
 */
#define VAL_GPIO0DIR            0x00000000
#define VAL_GPIO0DATA           0x00000000

/*
 * GPIO 1 initial setup.
 */
#define VAL_GPIO1DIR            PAL_PORT_BIT(GPIO1_LED1)   |                 \
                                PAL_PORT_BIT(GPIO1_LED2)   |                 \
                                PAL_PORT_BIT(GPIO1_BUZZER)

#define VAL_GPIO1DATA           PAL_PORT_BIT(GPIO1_LED1)


/*
 * GPIO 2 initial setup.
 */
#define VAL_GPIO2DIR            PAL_PORT_BIT(GPIO2_LCD_DC)   |               \
                                PAL_PORT_BIT(GPIO2_LCD_SS)   |               \
                                PAL_PORT_BIT(GPIO2_LCD_RES)
#define VAL_GPIO2DATA           PAL_PORT_BIT(GPIO2_LCD_SS)


/*
 * Pin definitions.
 */

#define GPIO1_LED1              5
#define GPIO1_LED2              4
#define GPIO1_SW_WAKEUP         3
#define GPIO1_BUZZER            6

#define GPIO2_SW_USER1          12
#define GPIO2_SW_USER2          11
#define GPIO2_SW_USER3          10
#define GPIO2_LCD_DC            15
#define GPIO2_LCD_SS            14
#define GPIO2_LCD_RES           13

/* LCD3310 pins */
#define LCD3310_RES_PIN   GPIO2_LCD_RES
#define LCD3310_RES_PORT  GPIO2
#define LCD3310_DC_PIN    GPIO2_LCD_DC
#define LCD3310_DC_PORT   GPIO2

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
