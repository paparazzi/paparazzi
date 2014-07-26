/*
 * Hey Emacs, this is a -*- makefile -*-
 *
 * Copyright (C) 2012 Piotr Esden-Tempski <piotr@esden.net>
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

/* Linker script for Lisa-M (STM32F105RCT6, 256K flash, 64K RAM). */

/* Define memory regions. */
MEMORY
{
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
	/* Leaving 2k of space at the end of rom for stored settings */
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 254K
}

/* Include the common ld script. */
INCLUDE libopencm3_stm32f1.ld

