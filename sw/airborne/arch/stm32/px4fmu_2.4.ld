/*
 * Copyright (C) 2016 The Paparazzi Team>
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
 */

/* Linker script for Pixhawk PX4FMU v2.4 (STM32F427, 1024K flash, 192K RAM). */

/* Define memory regions. */
MEMORY
{
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 192K
    /* Pixhawk seems to have had a bad badge of f4's, so only use the first 1mb */
    rom (rx) : ORIGIN = 0x08004000, LENGTH = 1008K
}

/* Include the common ld script. */
INCLUDE libopencm3_stm32f4.ld
