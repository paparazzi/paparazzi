/*
 * Paparazzi stm32 persistent flash low level routines
 *
 * Copyright (C) 2011 Martin Mueller <martinmm@pfump.org>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */


/*
 
  TODO:
  - remove last sector from usable flash memory in STM32 linker script
  
  flash data is located in the last page/sector of flash
  
  data_begin    flash_info.addr
  data_size     flash_info.addr + flash_info.size - 8
  checksum      flash_info.addr + flash_info.size - 4
*/

#ifndef STM32_SUBSYSTEMS_SETTINGS_H
#define STM32_SUBSYSTEMS_SETTINGS_H



#endif /* STM32_SUBSYSTEMS_SETTINGS_H */
