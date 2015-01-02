/*
 *
 * Copyright (C) 2009-2013 The Paparazzi Team
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
 * @file boards/ardrone/electrical_raw.c
 * arch specific electrical status readings
 */

#include "electrical_raw.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include "mcu_periph/i2c_smbus.h"
#include "subsystems/commands.h"
#include "generated/airframe.h"

struct Electrical electrical;

#if defined ADC_CHANNEL_VSUPPLY || defined ADC_CHANNEL_CURRENT || defined MILLIAMP_AT_FULL_THROTTLE
static struct {
#ifdef ADC_CHANNEL_VSUPPLY
  struct adc_buf vsupply_adc_buf;
#endif
#ifdef ADC_CHANNEL_CURRENT
  struct adc_buf current_adc_buf;
#endif
#ifdef MILLIAMP_AT_FULL_THROTTLE
  float nonlin_factor;
#endif
} electrical_priv;
#endif

#if defined COMMAND_THROTTLE
#define COMMAND_CURRENT_ESTIMATION COMMAND_THROTTLE
#elif defined COMMAND_THRUST
#define COMMAND_CURRENT_ESTIMATION COMMAND_THRUST
#endif

#ifndef CURRENT_ESTIMATION_NONLINEARITY
#define CURRENT_ESTIMATION_NONLINEARITY 1.2
#endif

int fd;

void electrical_init(void)
{
  // First we try to kill the program.elf and its respawner if it is running (done here because initializes first)
  int ret = system("killall -9 program.elf.respawner.sh; killall -9 program.elf");
  (void) ret;

  // Initialize 12c device for power
  fd = open("/dev/i2c-1", O_RDWR);
  if (ioctl(fd, I2C_SLAVE_FORCE, 0x4a) < 0) {
    fprintf(stderr, "Failed to set slave address: %m\n");
  }

  electrical_setup();
  electrical_priv.nonlin_factor = CURRENT_ESTIMATION_NONLINEARITY;
}

void electrical_setup(void)
{
  // Turn on MADC in CTRL1
  if (i2c_smbus_write_byte_data(fd, 0x00, 0x01))   {
    fprintf(stderr, "Failed to write to I2C device. 1\n");
  }
  // Select ADCIN0 for conversion in SW1SELECT_LSB
  if (i2c_smbus_write_byte_data(fd, 0x06, 0xff)) {
    fprintf(stderr, "Failed to write to I2C device. 2\n");
  }
  // Select ADCIN12 for conversion in SW1SELECT_MSB
  if (i2c_smbus_write_byte_data(fd, 0x07, 0xff))  {
    fprintf(stderr, "Failed to write to I2C device. 3\n");
  }
  // Setup register for averaging
  if (i2c_smbus_write_byte_data(fd, 0x08, 0xff))  {
    fprintf(stderr, "Failed to write to I2C device. 4\n");
  }
  // Start all channel conversion by setting bit 5 to one in CTRL_SW1
  if (i2c_smbus_write_byte_data(fd, 0x12, 0x20))  {
    fprintf(stderr, "Failed to write to I2C device. 5\n");
  }
}

void electrical_periodic(void)
{

  electrical_setup();

  unsigned char lsb, msb;
  lsb = i2c_smbus_read_byte_data(fd, 0x37);
  msb = i2c_smbus_read_byte_data(fd, 0x38);

  int raw_voltage = (lsb >> 6) | (msb << 2);

  // we know from spec sheet that ADCIN0 has no prescaler
  // so that the max voltage range is 1.5 volt
  // multiply by ten to get decivolts

  //from raw measurement we got quite a lineair response
  //9.0V=662, 9.5V=698, 10.0V=737,10.5V=774, 11.0V=811, 11.5V=848, 12.0V=886, 12.5V=923
  //leading to our 0.13595166 magic number for decivolts conversion
  electrical.vsupply = raw_voltage * 0.13595166;

  /*
   * Superellipse: abs(x/a)^n + abs(y/b)^n = 1
   * with a = 1
   * b = mA at full throttle
   * n = 1.2     This defines nonlinearity (1 = linear)
   * x = throttle
   * y = current
   *
   * define CURRENT_ESTIMATION_NONLINEARITY in your airframe file to change the default nonlinearity factor of 1.2
   */
  float b = (float)MILLIAMP_AT_FULL_THROTTLE;
  float x = ((float)commands[COMMAND_CURRENT_ESTIMATION]) / ((float)MAX_PPRZ);
  /* electrical.current y = ( b^n - (b* x/a)^n )^1/n
   * a=1, n = electrical_priv.nonlin_factor
   */
  electrical.current = b - pow((pow(b, electrical_priv.nonlin_factor) - pow((b * x), electrical_priv.nonlin_factor)),
                               (1. / electrical_priv.nonlin_factor));
}
