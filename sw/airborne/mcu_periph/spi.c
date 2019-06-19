/*
 * Copyright (C) 2005-2012 The Paparazzi Team
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

/**
 * @file mcu_periph/spi.c
 *
 * Architecture independent SPI (Serial Peripheral Interface) API.
 */

#include "std.h"
#include "mcu_periph/spi.h"

#if SPI_MASTER

#if USE_SPI0
struct spi_periph spi0;

void spi0_init(void)
{
  spi_init(&spi0);
  spi0_arch_init();
}
#endif // USE_SPI0


#if USE_SPI1
struct spi_periph spi1;

void spi1_init(void)
{
  spi_init(&spi1);
  spi1_arch_init();
}
#endif // USE_SPI1


#if USE_SPI2
struct spi_periph spi2;

void spi2_init(void)
{
  spi_init(&spi2);
  spi2_arch_init();
}
#endif // USE_SPI2


#if USE_SPI3
struct spi_periph spi3;

void spi3_init(void)
{
  spi_init(&spi3);
  spi3_arch_init();
}
#endif // USE_SPI3


#if USE_SPI4
struct spi_periph spi4;

void spi4_init(void)
{
  spi_init(&spi4);
  spi4_arch_init();
}
#endif // USE_SPI4


void spi_init(struct spi_periph *p)
{
  p->trans_insert_idx = 0;
  p->trans_extract_idx = 0;
  p->status = SPIIdle;
  p->mode = SPIMaster;
  p->suspend = false;
}

#endif /* SPI_MASTER */


#if SPI_SLAVE

#if USE_SPI0_SLAVE
struct spi_periph spi0;

void spi0_slave_init(void)
{
  spi_slave_init(&spi0);
  spi0_slave_arch_init();
}
#endif // USE_SPI0_SLAVE


#if USE_SPI1_SLAVE
struct spi_periph spi1;

void spi1_slave_init(void)
{
  spi_slave_init(&spi1);
  spi1_slave_arch_init();
}
#endif // USE_SPI1_SLAVE


#if USE_SPI2_SLAVE
struct spi_periph spi2;

void spi2_slave_init(void)
{
  spi_slave_init(&spi2);
  spi2_slave_arch_init();
}
#endif // USE_SPI2_SLAVE


#if USE_SPI3_SLAVE
struct spi_periph spi3;

void spi3_slave_init(void)
{
  spi_slave_init(&spi3);
  spi3_slave_arch_init();
}
#endif // USE_SPI3_SLAVE


extern void spi_slave_init(struct spi_periph *p)
{
  p->trans_insert_idx = 0;
  p->trans_extract_idx = 0;
  p->status = SPIIdle;
  p->mode = SPISlave;
  p->suspend = false;
}

#endif /* SPI_SLAVE */

