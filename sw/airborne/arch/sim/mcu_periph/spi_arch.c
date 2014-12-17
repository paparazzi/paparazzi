/*
 * Copyright (C) 2012 Felix Ruess <felix.ruess@gmail.com>
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
 */

/**
 * @file arch/sim/mcu_periph/spi_arch.c
 * Dummy functions for handling of SPI hardware in sim.
 */

#include "mcu_periph/spi.h"


bool_t spi_submit(struct spi_periph *p __attribute__((unused)), struct spi_transaction *t __attribute__((unused))) { return TRUE;}

void spi_init_slaves(void) {}

void spi_slave_select(uint8_t slave __attribute__((unused))) {}

void spi_slave_unselect(uint8_t slave __attribute__((unused))) {}

bool_t spi_lock(struct spi_periph *p __attribute__((unused)), uint8_t slave __attribute__((unused))) { return TRUE; }

bool_t spi_resume(struct spi_periph *p __attribute__((unused)), uint8_t slave __attribute__((unused))) { return TRUE; }

