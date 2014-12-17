/*
 * Copyright (C) 2009-20010 Paparazzi Team
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

#ifndef FMS_SPI_LINK_H
#define FMS_SPI_LINK_H

#include <inttypes.h>
#include <unistd.h>

struct SpiLink {
  int      fd;
  char    *device;
  uint8_t  mode;
  uint8_t  bits;
  uint32_t speed;
  uint16_t delay;
  /* number of message exchanged since initialization */
  uint32_t msg_cnt;
  /* number of crc errors on received messages        */
  uint32_t crc_err_cnt;
};

struct SpiLink spi_link;

/*
 * initialize peripheral
 */
extern int spi_link_init(void);

/*
 *  exchange a data buffer
 *  the last byte of buf_out will be overwiten with a crc
 *  the last byte of buf_in  will contain the received crc
 *  count is the size of buf_out and buf_in, that is
 *  the count of data to exchange+1
 */
extern int spi_link_send(void *buf_out, size_t count, void *buf_in, uint8_t *crc_valid);

/*
 * just for debuging purposes
 */
extern uint8_t crc_calc_block_crc8(const uint8_t buf[], uint32_t len);


#endif /* FMS_SPI_LINK_H */
