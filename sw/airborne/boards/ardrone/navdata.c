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

#include "std.h"
#include "navdata.h"
#include "subsystems/ins.h"

#define NAVDATA_PACKET_SIZE 60
#define NAVDATA_START_BYTE 0x3a

static inline bool_t acquire_baro_calibration(void);
static void navdata_cropbuffer(int cropsize);

navdata_port nav_port;
static int nav_fd = 0;
static int16_t previousUltrasoundHeight;
measures_t navdata;

#include "subsystems/sonar.h"
uint16_t sonar_meas = 0;


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

#if DOWNLINK
#include "subsystems/datalink/telemetry.h"

static void send_navdata(void) {
  DOWNLINK_SEND_ARDRONE_NAVDATA(DefaultChannel, DefaultDevice,
      &navdata.taille,
      &navdata.nu_trame,
      &navdata.ax,
      &navdata.ay,
      &navdata.az,
      &navdata.vx,
      &navdata.vy,
      &navdata.vz,
      &navdata.temperature_acc,
      &navdata.temperature_gyro,
      &navdata.ultrasound,
      &navdata.us_debut_echo,
      &navdata.us_fin_echo,
      &navdata.us_association_echo,
      &navdata.us_distance_echo,
      &navdata.us_curve_time,
      &navdata.us_curve_value,
      &navdata.us_curve_ref,
      &navdata.nb_echo,
      &navdata.sum_echo,
      &navdata.gradient,
      &navdata.flag_echo_ini,
      &navdata.pressure,
      &navdata.temperature_pressure,
      &navdata.mx,
      &navdata.my,
      &navdata.mz,
      &navdata.chksum,
      &nav_port.checksum_errors);
}
#endif

bool_t navdata_init()
{
  if (nav_fd <= 0) {
    nav_fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (nav_fd == -1) {
      perror("navdata_init: Unable to open /dev/ttyO1 - ");
      return FALSE;
    }
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

  // read some potential dirt (retry alot of times)
  char tmp[100];
  for(int i = 0; i < 100; i++) {
    uint16_t dirt = read(nav_fd, tmp, sizeof tmp);
    (void) dirt;

    cmd=0x02;
    navdata_write(&cmd, 1);
    usleep(200);
  }

  baro_calibrated = FALSE;
  if(!acquire_baro_calibration())
    return FALSE;

  // start acquisition
  cmd = 0x01;
  navdata_write(&cmd, 1);

  navdata_imu_available = FALSE;
  navdata_baro_available = FALSE;

  previousUltrasoundHeight = 0;
  nav_port.checksum_errors = 0;
  nav_port.bytesRead = 0;
  nav_port.totalBytesRead = 0;
  nav_port.packetsRead = 0;
  nav_port.isInitialized = TRUE;

#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, "ARDRONE_NAVDATA", send_navdata);
#endif

  return TRUE;
}

static inline bool_t acquire_baro_calibration(void)
{
  // start baro calibration acquisition
  uint8_t cmd=0x17; // send cmd 23
  navdata_write(&cmd, 1);

  uint8_t calibBuffer[22];

  if (full_read(nav_fd, calibBuffer, sizeof calibBuffer) < 0)
  {
    perror("acquire_baro_calibration: read failed");
    return FALSE;
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

  baro_calibrated = TRUE;
  return TRUE;
}

void navdata_read()
{
  int newbytes = read(nav_fd, nav_port.buffer+nav_port.bytesRead, NAVDATA_BUFFER_SIZE-nav_port.bytesRead);

  // because non-blocking read returns -1 when no bytes available
  if (newbytes > 0)
  {
    nav_port.bytesRead += newbytes;
    nav_port.totalBytesRead += newbytes;
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
    temp_or_press_was_updated_last = TRUE;

    // This means that press must remain constant
    if (lastpressval != 0)
    {
      // If pressure was updated: this is a sync error
      if (lastpressval != navdata.pressure)
      {
        // wait for temp again
        temp_or_press_was_updated_last = FALSE;
        sync_errors++;
        navdata_baro_available = TRUE;
      }
    }
  }
  else
  {
    // press was updated
    temp_or_press_was_updated_last = FALSE;

    // This means that temp must remain constant
    if (lasttempval != 0)
    {
      // If temp was updated: this is a sync error
      if (lasttempval != navdata.temperature_pressure)
      {
        // wait for press again
        temp_or_press_was_updated_last = TRUE;
        sync_errors++;
      }
    }

    navdata_baro_available = TRUE;
  }

  lastpressval = navdata.pressure;
  lasttempval = navdata.temperature_pressure;
}

void navdata_update()
{
  static bool_t last_checksum_wrong = FALSE;
  // Check if initialized
  if (!nav_port.isInitialized) {
    navdata_init();
    return;
  }

  // Start reading
  navdata_read();

  // while there is something interesting to do...
  while (nav_port.bytesRead >= NAVDATA_PACKET_SIZE)
  {
    if (nav_port.buffer[0] == NAVDATA_START_BYTE)
    {
      assert(sizeof navdata == NAVDATA_PACKET_SIZE);
      memcpy(&navdata, nav_port.buffer, NAVDATA_PACKET_SIZE);

      // Calculating the checksum
      uint16_t checksum = 0;
      for(int i = 2; i < NAVDATA_PACKET_SIZE-2; i += 2) {
        checksum += nav_port.buffer[i] + (nav_port.buffer[i+1] << 8);
      }

      // When checksum is incorrect
      if(navdata.chksum != checksum) {
        printf("Checksum error [calculated: %d] [packet: %d] [diff: %d]\n",checksum , navdata.chksum, checksum-navdata.chksum);
        nav_port.checksum_errors++;
      }

      // When we already dropped a packet or checksum is correct
      if(last_checksum_wrong || navdata.chksum == checksum) {
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

#ifdef USE_SONAR
        if (navdata.ultrasound < 10000)
        {
            sonar_meas = navdata.ultrasound;
            ins_update_sonar();

        }
#endif


        navdata_imu_available = TRUE;
        last_checksum_wrong = FALSE;
        nav_port.packetsRead++;
      }

      // Crop the buffer
      navdata_cropbuffer(NAVDATA_PACKET_SIZE);
    }
    else
    {
      // find start byte, copy all data from startbyte to buffer origin, update bytesread
      uint8_t * pint;
      pint = memchr(nav_port.buffer, NAVDATA_START_BYTE, nav_port.bytesRead);

      if (pint != NULL) {
        navdata_cropbuffer(pint - nav_port.buffer);
      } else {
        // if the start byte was not found, it means there is junk in the buffer
        nav_port.bytesRead = 0;
      }
    }
  }
}

int16_t navdata_height(void) {
  if (navdata.ultrasound > 10000) {
    return previousUltrasoundHeight;
  }

  int16_t ultrasoundHeight = 0;
  ultrasoundHeight = (navdata.ultrasound - 880) / 26.553;
  previousUltrasoundHeight = ultrasoundHeight;
  return ultrasoundHeight;
}

static void navdata_cropbuffer(int cropsize)
{
  if (nav_port.bytesRead - cropsize < 0) {
    // TODO think about why the amount of bytes read minus the cropsize gets below zero
    printf("BytesRead(=%d) - Cropsize(=%d) may not be below zero. port->buffer=%p\n", nav_port.bytesRead, cropsize, nav_port.buffer);
    return;
  }

  memmove(nav_port.buffer, nav_port.buffer+cropsize, NAVDATA_BUFFER_SIZE-cropsize);
  nav_port.bytesRead -= cropsize;
}
