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
 * @file modules/sensors/bat_voltage_ardrone2.c
 * Read battery voltage on ARDrone2 and publish to electrical subsystem.
 */


#include "modules/sensors/bat_voltage_ardrone2.h"

#include <stdio.h>
#include <string.h>
//#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include "mcu_periph/i2c_smbus.h"

#include "subsystems/electrical.h"



void electrical_ardrone2_setup(void);

int fd;

void bat_voltage_ardrone2_init(void)
{
  // Initialize 12c device for power
  fd = open("/dev/i2c-1", O_RDWR);
  if (ioctl(fd, I2C_SLAVE_FORCE, 0x4a) < 0) {
    fprintf(stderr, "Failed to set slave address: %m\n");
  }

  electrical_ardrone2_setup();

}

void electrical_ardrone2_setup(void)
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

void bat_voltage_ardrone2_periodic(void)
{

  electrical_ardrone2_setup();

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

}
