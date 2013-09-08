/*
 * Copyright (C) 2013 Dino Hensen, Vincent van Hoek
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

/**
 * @file boards/ardrone/navdata.c
 * ardrone2 navdata aquisition driver.
 *
 * The ardrone2 provides a navdata stream of packets
 * containing info about all sensors at a rate of 200Hz.
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>     // for O_RDWR, O_NOCTTY, O_NONBLOCK
#include <termios.h>   // for baud rates and options
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <assert.h>
#include "navdata.h"

#define NAVDATA_PACKET_SIZE 60
#define NAVDATA_BUFFER_SIZE 80
#define NAVDATA_START_BYTE 0x3a

typedef struct {
	uint8_t isInitialized;
	uint8_t isOpen;
	uint16_t bytesRead;
	uint32_t totalBytesRead;
	uint32_t packetsRead;
	uint8_t buffer[NAVDATA_BUFFER_SIZE];
} navdata_port;

static navdata_port port;
static int nav_fd;

measures_t navdata;

// FIXME(ben): there must be a better home for these
ssize_t full_write(int fd, const uint8_t *buf, size_t count)
{
  size_t written = 0;

  while(written < count)
  {
    ssize_t n = write(fd, buf + written, count - written);
    if (n < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
	continue;
      return n;
    }
    written += n;
  }
  return written;
}

ssize_t full_read(int fd, uint8_t *buf, size_t count)
{
  // Apologies for illiteracy, but we can't overload |read|.
  size_t readed = 0;

  while(readed < count)
  {
    ssize_t n = read(fd, buf + readed, count - readed);
    if (n < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
	continue;
      return n;
    }
    readed += n;
  }
  return readed;
}

static void navdata_write(const uint8_t *buf, size_t count)
{
  if (full_write(nav_fd, buf, count) < 0)
    perror("navdata_write: Write failed");
}

int navdata_init()
{
  nav_fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (nav_fd == -1)
  {
    perror("navdata_init: Unable to open /dev/ttyO1 - ");
    return 1;
  } else {
    port.isOpen = 1;
  }

  fcntl(nav_fd, F_SETFL, 0); //read calls are non blocking
  //set port options
  struct termios options;
  //Get the current options for the port
  tcgetattr(nav_fd, &options);
  //Set the baud rates to 460800
  cfsetispeed(&options, B460800);
  cfsetospeed(&options, B460800);

  options.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode
  options.c_iflag = 0; //clear input options
  options.c_lflag = 0; //clear local options
  options.c_oflag &= ~OPOST; //clear output options (raw output)

  //Set the new options for the port
  tcsetattr(nav_fd, TCSANOW, &options);

  // stop acquisition
  uint8_t cmd=0x02;
  navdata_write(&cmd, 1);

  baro_calibrated = 0;
  acquire_baro_calibration();

  // start acquisition
  cmd=0x01;
  navdata_write(&cmd, 1);

  navdata_imu_available = 0;
  navdata_baro_available = 0;

  port.bytesRead = 0;
  port.totalBytesRead = 0;
  port.packetsRead = 0;
  port.isInitialized = 1;

  previousUltrasoundHeight = 0;

  return 0;
}

void acquire_baro_calibration()
{
  char tmp[100];
  read(nav_fd, tmp, sizeof tmp); // read some potential dirt

  // start baro calibration acquisition
  uint8_t cmd=0x17; // send cmd 23
  navdata_write(&cmd, 1);

  uint8_t calibBuffer[22];

  if (full_read(nav_fd, calibBuffer, sizeof calibBuffer) < 0)
  {
    perror("acquire_baro_calibration: read failed");
    return;
  }

  baro_calibration.ac1 = calibBuffer[0]  << 8 | calibBuffer[1];
  baro_calibration.ac2 = calibBuffer[2]  << 8 | calibBuffer[3];
  baro_calibration.ac3 = calibBuffer[4]  << 8 | calibBuffer[5];
  baro_calibration.ac4 = calibBuffer[6]  << 8 | calibBuffer[7];
  baro_calibration.ac5 = calibBuffer[8]  << 8 | calibBuffer[9];
  baro_calibration.ac6 = calibBuffer[10] << 8 | calibBuffer[11];
  baro_calibration.b1  = calibBuffer[12] << 8 | calibBuffer[13];
  baro_calibration.b2  = calibBuffer[14] << 8 | calibBuffer[15];
  baro_calibration.mb  = calibBuffer[16] << 8 | calibBuffer[17];
  baro_calibration.mc  = calibBuffer[18] << 8 | calibBuffer[19];
  baro_calibration.md  = calibBuffer[20] << 8 | calibBuffer[21];

  printf("Calibration AC1: %d\n", baro_calibration.ac1);
  printf("Calibration AC2: %d\n", baro_calibration.ac2);
  printf("Calibration AC3: %d\n", baro_calibration.ac3);
  printf("Calibration AC4: %d\n", baro_calibration.ac4);
  printf("Calibration AC5: %d\n", baro_calibration.ac5);
  printf("Calibration AC6: %d\n", baro_calibration.ac6);

  printf("Calibration B1: %d\n", baro_calibration.b1);
  printf("Calibration B2: %d\n", baro_calibration.b2);

  printf("Calibration MB: %d\n", baro_calibration.mb);
  printf("Calibration MC: %d\n", baro_calibration.mc);
  printf("Calibration MD: %d\n", baro_calibration.md);

  baro_calibrated = 1;
}

void navdata_close()
{
  port.isOpen = 0;
  close(nav_fd);
}

void navdata_read()
{
  int newbytes = 0;

  if (port.isInitialized != 1)
    navdata_init();

  if (port.isOpen != 1)
    return;

  newbytes = read(nav_fd, port.buffer+port.bytesRead, NAVDATA_BUFFER_SIZE-port.bytesRead);

  // because non-blocking read returns -1 when no bytes available
  if (newbytes > 0)
  {
    port.bytesRead += newbytes;
    port.totalBytesRead += newbytes;
  }
}

static void baro_update_logic(void)
{
  static int32_t lastpressval = 0;
  static uint16_t lasttempval = 0;
  static uint8_t temp_or_press_was_updated_last = 0; // 0 = press, so we now wait for temp, 1 = temp so we now wait for press

  static int sync_errors;

  if (temp_or_press_was_updated_last == 0)  // Last update was press so we are now waiting for temp
  {
    // temp was updated
    temp_or_press_was_updated_last = 1;

    // This means that press must remain constant
    if (lastpressval != 0)
    {
      // If pressure was updated: this is a sync error
      if (lastpressval != navdata.pressure)
      {
        // wait for temp again
        temp_or_press_was_updated_last = 0;
        sync_errors++;
        navdata_baro_available = 1;
      }
    }
  }
  else
  {
    // press was updated
    temp_or_press_was_updated_last = 0;

    // This means that temp must remain constant
    if (lasttempval != 0)
    {
      // If temp was updated: this is a sync error
      if (lasttempval != navdata.temperature_pressure)
      {
        // wait for press again
        temp_or_press_was_updated_last = 1;
        sync_errors++;
      }
    }

    navdata_baro_available = 1;
  }

  lastpressval = navdata.pressure;
  lasttempval = navdata.temperature_pressure;

  // debug
  // navdata->temperature_pressure = sync_errors;
}

void navdata_update()
{
  navdata_read();

  // while there is something interesting to do...
  while (port.bytesRead >= NAVDATA_PACKET_SIZE)
  {
    if (port.buffer[0] == NAVDATA_START_BYTE)
    {
      // if checksum is OK
      if ( 1 ) // we dont know how to calculate the checksum
//      if ( navdata_checksum() == 0 )
      {
	assert(sizeof navdata == NAVDATA_PACKET_SIZE);
        memcpy(&navdata, port.buffer, NAVDATA_PACKET_SIZE);

        // Invert byte order so that TELEMETRY works better
        uint8_t tmp;
        uint8_t* p = (uint8_t*) &(navdata.pressure);
        tmp = p[0];
        p[0] = p[1];
        p[1] = tmp;
        p = (uint8_t*) &(navdata.temperature_pressure);
        tmp = p[0];
        p[0] = p[1];
        p[1] = tmp;

        baro_update_logic();

        navdata_imu_available = 1;

        port.packetsRead++;
//        printf("CCRC=%d, GCRC=%d, error=%d\n", crc, navdata->chksum, abs(crc-navdata->chksum));
        //navdata_getHeight();
      }
      navdata_CropBuffer(NAVDATA_PACKET_SIZE);
    }
    else
    {
      // find start byte, copy all data from startbyte to buffer origin, update bytesread
      uint8_t * pint;
      pint = memchr(port.buffer, NAVDATA_START_BYTE, port.bytesRead);

      if (pint != NULL) {
        navdata_CropBuffer(pint - port.buffer);
      } else {
        // if the start byte was not found, it means there is junk in the buffer
        port.bytesRead = 0;
      }
    }
  }
}

void navdata_CropBuffer(int cropsize)
{
  if (port.bytesRead - cropsize < 0) {
    // TODO think about why the amount of bytes read minus the cropsize gets below zero
    printf("BytesRead(=%d) - Cropsize(=%d) may not be below zero. port->buffer=%p\n", port.bytesRead, cropsize, port.buffer);
    return;
  }

  memmove(port.buffer, port.buffer+cropsize, NAVDATA_BUFFER_SIZE-cropsize);
  port.bytesRead -= cropsize;
}

int16_t navdata_getHeight() {

  if (navdata.ultrasound > 10000) {
    return previousUltrasoundHeight;
  }

  int16_t ultrasoundHeight = 0;

  ultrasoundHeight = (navdata.ultrasound - 880) / 26.553;

  previousUltrasoundHeight = ultrasoundHeight;

  return ultrasoundHeight;
}

// The checksum should be calculated here: we don't know the algorithm
uint16_t navdata_checksum() {
  navdata_cks = 0;
  navdata_cks += navdata.nu_trame;
  navdata_cks += navdata.ax;
  navdata_cks += navdata.ay;
  navdata_cks += navdata.az;
  navdata_cks += navdata.vx;
  navdata_cks += navdata.vy;
  navdata_cks += navdata.vz;
  navdata_cks += navdata.temperature_acc;
  navdata_cks += navdata.temperature_gyro;
  navdata_cks += navdata.ultrasound;
  navdata_cks += navdata.us_debut_echo;
  navdata_cks += navdata.us_fin_echo;
  navdata_cks += navdata.us_association_echo;
  navdata_cks += navdata.us_distance_echo;
  navdata_cks += navdata.us_curve_time;
  navdata_cks += navdata.us_curve_value;
  navdata_cks += navdata.us_curve_ref;
  navdata_cks += navdata.nb_echo;
  navdata_cks += navdata.sum_echo;
  navdata_cks += navdata.gradient;
  navdata_cks += navdata.flag_echo_ini;
  navdata_cks += navdata.pressure;
  navdata_cks += navdata.temperature_pressure;
  navdata_cks += navdata.mx;
  navdata_cks += navdata.my;
  navdata_cks += navdata.mz;
//  navdata_cks += navdata->chksum;

  return 0; // we dont know how to calculate the checksum
}
