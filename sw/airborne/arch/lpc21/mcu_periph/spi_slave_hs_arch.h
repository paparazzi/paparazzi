/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 * @file arch/lpc21/mcu_periph/spi_slave_hs_arch.h
 * @ingroup lpc21_arch
 *
 * Highspeed SPI Slave Interface.
 * SS on P0.20
 * Circular Buffer
 */

#ifndef SPI_SLAVE_HS_ARCH_H
#define SPI_SLAVE_HS_ARCH_H

#include "std.h"
#include "pprzlink/pprzlink_device.h"

struct spi_slave_hs {
  /** Generic device interface */
  struct link_device device;
};

extern struct spi_slave_hs spi_slave_hs;

#define SpiEnable() {   \
    SetBit(SSPCR1, SSE);  \
  }

#define SpiDisable() {    \
    ClearBit(SSPCR1, SSE);  \
  }


#define SPI_SLAVE_HS_RX_BUFFER_SIZE 256

extern uint16_t spi_slave_hs_rx_insert_idx, spi_slave_hs_rx_extract_idx;
extern uint8_t spi_slave_hs_rx_buffer[SPI_SLAVE_HS_RX_BUFFER_SIZE];


#define SPI_SLAVE_HS_TX_BUFFER_SIZE 64

extern uint8_t spi_slave_hs_tx_insert_idx, spi_slave_hs_tx_extract_idx;
extern uint8_t spi_slave_hs_tx_buffer[SPI_SLAVE_HS_TX_BUFFER_SIZE];


#endif
