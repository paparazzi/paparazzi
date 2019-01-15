/*
 * Copyright (C) 2014 Felix Ruess <felix.ruess@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file arch/linux/mcu_periph/spi_arch.c
 * Handling of SPI hardware for Linux.
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "mcu_periph/spi.h"
#include BOARD_CONFIG


void spi_init_slaves(void)
{
  /* for now we assume that each SPI device has it's SLAVE CS already set up
   * e.g. in pin muxing of BBB
   */
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
bool spi_submit(struct spi_periph *p, struct spi_transaction *t)
{
  int fd = (int)p->reg_addr;

  struct spi_ioc_transfer xfer;
  memset(&xfer, 0, sizeof xfer);

  /* length in bytes of transaction */
  uint16_t buf_len = Max(t->input_length, t->output_length);

  // temp buffers, used if necessary
  uint8_t tx_buf[buf_len], rx_buf[buf_len];
  memset(tx_buf, 0, buf_len);
  memset(rx_buf, 0, buf_len);

  /* handle transactions with different input/output length */
  if (buf_len > t->output_length) {
    /* copy bytes to transmit to larger buffer, rest filled with zero */
    memcpy(tx_buf, (void *)t->output_buf, t->output_length);
    xfer.tx_buf = (unsigned long)tx_buf;
  } else {
    xfer.tx_buf = (unsigned long)t->output_buf;
  }

  if (buf_len > t->input_length) {
    xfer.rx_buf = (unsigned long)rx_buf;
  } else {
    xfer.rx_buf = (unsigned long)t->input_buf;
  }

  xfer.len = buf_len;
  /* fixed speed of 1Mhz for now, use SPIClockDiv?? */
  xfer.speed_hz = (uint32_t)p->init_struct;
  xfer.delay_usecs = 0;
  if (t->dss == SPIDss16bit) {
    xfer.bits_per_word = 16;
  } else {
    xfer.bits_per_word = 8;
  }
  if (t->select == SPISelectUnselect || t->select == SPIUnselect) {
    xfer.cs_change = 1;
  }

  if (ioctl(fd, SPI_IOC_MESSAGE(1), &xfer) < 0) {
    t->status = SPITransFailed;
    return false;
  }

  /* copy received data if we had to use an extra rx_buffer */
  if (buf_len > t->input_length) {
    memcpy((void *)t->input_buf, rx_buf, t->input_length);
  }

  t->status = SPITransSuccess;
  return true;
}
#pragma GCC diagnostic pop

bool spi_lock(struct spi_periph *p, uint8_t slave)
{
  // not implemented
  return false;
}

bool spi_resume(struct spi_periph *p, uint8_t slave)
{
  // not implemented
  return false;
}


#if USE_SPI0

#ifndef SPI0_MODE
#define SPI0_MODE (SPI_CPOL | SPI_CPHA)
#endif

#ifndef SPI0_LSB_FIRST
#define SPI0_LSB_FIRST 0
#endif

#ifndef SPI0_BITS_PER_WORD
#define SPI0_BITS_PER_WORD 8
#endif

#ifndef SPI0_MAX_SPEED_HZ
#define SPI0_MAX_SPEED_HZ 1000000
#endif

void spi0_arch_init(void)
{
  int fd = open("/dev/spidev1.0", O_RDWR);

  if (fd < 0) {
    perror("Could not open SPI device /dev/spidev1.0");
    spi0.reg_addr = NULL;
    return;
  }
  spi0.reg_addr = (void *)fd;

  /* spi mode */
  unsigned char spi_mode = SPI0_MODE;
  if (ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) < 0) {
    perror("SPI0: can't set spi mode");
  }

  /* set to MSB first */
  unsigned char spi_order = SPI0_LSB_FIRST;
  if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &spi_order) < 0) {
    perror("SPI0: can't set spi bit justification");
  }

  /* bits per word default to 8 */
  unsigned char spi_bits_per_word = SPI0_BITS_PER_WORD;
  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word) < 0) {
    perror("SPI0: can't set bits per word");
  }

  /* max speed in hz, 1MHz for now */
  unsigned int spi_speed = SPI0_MAX_SPEED_HZ;
  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) {
    perror("SPI0: can't set max speed hz");
  }
  spi0.init_struct = (void *)SPI0_MAX_SPEED_HZ;
}
#endif /* USE_SPI0 */

#if USE_SPI1

#ifndef SPI1_MODE
#define SPI1_MODE (SPI_CPOL | SPI_CPHA)
#endif

#ifndef SPI1_LSB_FIRST
#define SPI1_LSB_FIRST 0
#endif

#ifndef SPI1_BITS_PER_WORD
#define SPI1_BITS_PER_WORD 8
#endif

#ifndef SPI1_MAX_SPEED_HZ
#define SPI1_MAX_SPEED_HZ 1000000
#endif

void spi1_arch_init(void)
{
  int fd = open("/dev/spidev1.1", O_RDWR);

  if (fd < 0) {
    perror("Could not open SPI device /dev/spidev1.1");
    spi1.reg_addr = NULL;
    return;
  }
  spi1.reg_addr = (void *)fd;

  /* spi mode */
  unsigned char spi_mode = SPI1_MODE;
  if (ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) < 0) {
    perror("SPI1: can't set spi mode");
  }

  /* set to MSB first */
  unsigned char spi_order = SPI1_LSB_FIRST;
  if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &spi_order) < 0) {
    perror("SPI1: can't set spi bit justification");
  }

  /* bits per word default to 8 */
  unsigned char spi_bits_per_word = SPI1_BITS_PER_WORD;
  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word) < 0) {r
    perror("SPI1: can't set bits per word");
  }

  /* max speed in hz, 1MHz for now */
  unsigned int spi_speed = SPI1_MAX_SPEED_HZ;
  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) {
    perror("SPI1: can't set max speed hz");
  }
  spi1.init_struct = (void *)SPI1_MAX_SPEED_HZ;
}
#endif /* USE_SPI1 */
