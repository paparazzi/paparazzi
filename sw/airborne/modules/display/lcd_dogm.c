/*
 * Copyright (C) 2010  Martin Mueller
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

/*
   initial version, needs polishing!

   3 line LCD at 3.3V

   EA DOGM163 <-> LPC2148 SPI1

   SI             MOSI
   CLK            SCK
   RS             DRDY (used to switch cmd/data)
   CSB            SSEL

*/

#include "display/lcd_dogm_hw.h"
#include "lcd_dogm.h"


void lcd_cmd(uint8_t command)
{
  uint32_t i;
  for (i = 0; i < 20000; i++);
  lcddogmSelect();
  lcddogmCmdMode();
  lcd_spi_tx(command);
}

void lcd_data(uint8_t data)
{
  uint32_t i;
  for (i = 0; i < 100000; i++);
  lcddogmSelect();
  lcddogmDataMode();
  lcd_spi_tx(data);
}

void lcd_dogm_init(void)
{
  uint32_t i;

  for (i = 0; i < 100000; i++);
  lcd_dogm_init_hw();

  /* Write configuration */
  lcd_cmd(DOGM_FUN_SET_1);
  lcd_cmd(DOGM_BIAS_SET);
  lcd_cmd(DOGM_PWR_CTRL);
  lcd_cmd(DOGM_FOLLOWER);
  lcd_cmd(DOGM_CONTRAST);
  lcd_cmd(DOGM_DISP_ON);
  lcd_cmd(DOGM_CLEAR);
  lcd_cmd(DOGM_ENTRY_MODE);

  /* sample data */
  lcd_data('C');
  lcd_data('i');
  lcd_data('r');
  lcd_data('c');
  lcd_data('l');
  lcd_data('e');
  lcd_data('C');
  lcd_data('W');
  lcd_data(' ');
  lcd_data('9');
  lcd_data('9');
  lcd_data(' ');
  lcd_data('1');
  lcd_data('0');
  lcd_data('.');
  lcd_data('3');

  lcd_data('1');
  lcd_data('2');
  lcd_data('.');
  lcd_data('6');
  lcd_data(' ');
  lcd_data('M');
  lcd_data(' ');
  lcd_data('1');
  lcd_data('2');
  lcd_data('+');
  lcd_data('3');
  lcd_data(' ');
  lcd_data('1');
  lcd_data('4');
  lcd_data('9');
  lcd_data('3');

  lcd_data('o');
  lcd_data('k');
  lcd_data(' ');
  lcd_data('3');
  lcd_data(' ');
  lcd_data('0');
  lcd_data('2');
  lcd_data('1');
  lcd_data(':');
  lcd_data('3');
  lcd_data('4');
  lcd_data(' ');
  lcd_data('1');
  lcd_data('5');
  lcd_data('0');
  lcd_data('0');
}

