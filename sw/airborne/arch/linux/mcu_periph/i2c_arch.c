/*
 *
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file arch/linux/mcu_periph/i2c_arch.c
 * I2C functionality
 */

#include "mcu_periph/i2c.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>

void i2c_event(void)
{
}

void i2c_setbitrate(struct i2c_periph *p  __attribute__((unused)), int bitrate __attribute__((unused)))
{
}

bool_t i2c_idle(struct i2c_periph *p __attribute__((unused)))
{
  return TRUE;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
bool_t i2c_submit(struct i2c_periph *p, struct i2c_transaction *t)
{
  int file = (int)p->reg_addr;

  // Set the slave address
  ioctl(file, I2C_SLAVE, t->slave_addr);

  // Switch the different transaction types
  switch (t->type) {
      // Just transmitting
    case I2CTransTx:
      if (write(file, (uint8_t *)t->buf, t->len_w) < 0) {
        t->status = I2CTransFailed;
        return TRUE;
      }
      break;
      // Just reading
    case I2CTransRx:
      if (read(file, (uint8_t *)t->buf, t->len_r) < 0) {
        t->status = I2CTransFailed;
        return TRUE;
      }
      break;
      // First Transmit and then read
    case I2CTransTxRx:
      if (write(file, (uint8_t *)t->buf, t->len_w) < 0 ||
          read(file, (uint8_t *)t->buf, t->len_r) < 0) {
        t->status = I2CTransFailed;
        return TRUE;
      }
      break;
    default:
      break;
  }

  // Successfull transfer
  t->status = I2CTransSuccess;
  return TRUE;
}
#pragma GCC diagnostic pop


#if USE_I2C0
struct i2c_errors i2c0_errors;

void i2c0_hw_init(void)
{
  i2c1.reg_addr = (void *)open("/dev/i2c-0", O_RDWR);
  i2c0.errors = &i2c0_errors;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c0_errors);
}
#endif

#if USE_I2C1
struct i2c_errors i2c1_errors;

void i2c1_hw_init(void)
{
  i2c1.reg_addr = (void *)open("/dev/i2c-1", O_RDWR);
  i2c1.errors = &i2c1_errors;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c1_errors);
}
#endif

#if USE_I2C2
struct i2c_errors i2c2_errors;

void i2c2_hw_init(void)
{
  i2c2.reg_addr = (void *)open("/dev/i2c-2", O_RDWR);
  i2c2.errors = &i2c2_errors;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c2_errors);
}
#endif
