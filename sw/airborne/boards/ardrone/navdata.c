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
#include "navdata.h"

int nav_fd;

int navdata_init()
{
  port = malloc(sizeof(navdata_port));

  nav_fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (nav_fd == -1)
  {
    perror("navdata_init: Unable to open /dev/ttyO1 - ");
    return 1;
  } else {
    port->isOpen = 1;
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
  write(nav_fd, &cmd, 1);

  baro_calibrated = 0;
  acquire_baro_calibration();

  // start acquisition
  cmd=0x01;
  write(nav_fd, &cmd, 1);

  navdata = malloc(sizeof(measures_t));
  navdata_imu_available = 0;
  navdata_baro_available = 0;

  port->bytesRead = 0;
  port->totalBytesRead = 0;
  port->packetsRead = 0;
  port->isInitialized = 1;

  previousUltrasoundHeight = 0;

  return 0;
}

void acquire_baro_calibration()
{
  read(nav_fd, NULL, 100); // read some potential dirt

  // start baro calibration acquisition
  uint8_t cmd=0x17; // send cmd 23
  write(nav_fd, &cmd, 1);

  uint8_t calibBuffer[22];
  int calib_bytes_read, calib_bytes_left, readcount;
  calib_bytes_read = 0;
  calib_bytes_left = 22;
  readcount = 0;

  while(calib_bytes_read != 22) {
    readcount = read(nav_fd, calibBuffer+(22-calib_bytes_left), calib_bytes_left);
    if (readcount > 0) {
      calib_bytes_read += readcount;
      calib_bytes_left -= readcount;
    }
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

  printf("Calibration bytes read: %d\n", calib_bytes_read);

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
  port->isOpen = 0;
  close(nav_fd);
}

void navdata_read()
{
  int newbytes = 0;

  if (port->isInitialized != 1)
    navdata_init();

  if (port->isOpen != 1)
    return;

  newbytes = read(nav_fd, port->buffer+port->bytesRead, NAVDATA_BUFFER_SIZE-port->bytesRead);

  // because non-blocking read returns -1 when no bytes available
  if (newbytes > 0)
  {
    port->bytesRead += newbytes;
    port->totalBytesRead += newbytes;
  }
}

void navdata_update()
{
  navdata_read();

  // while there is something interesting to do...
  while (port->bytesRead >= 60)
  {
    if (port->buffer[0] == NAVDATA_START_BYTE)
    {
      // if checksum is OK
      if ( 1 ) // we dont know how to calculate the checksum
//      if ( navdata_checksum() == 0 )
      {
        memcpy(navdata, port->buffer, NAVDATA_PACKET_SIZE);
        
        // Invert byte order so that TELEMETRY works better
        uint8_t tmp;
        uint8_t* p = (uint8_t*) &(navdata->pressure);
        tmp = p[0];
        p[0] = p[1];
        p[1] = tmp;
        p = (uint8_t*) &(navdata->temperature_pressure);
        tmp = p[0];
        p[0] = p[1];
        p[1] = tmp;

        navdata_imu_available = 1;
        navdata_baro_available = 1;

        port->packetsRead++;
//        printf("CCRC=%d, GCRC=%d, error=%d\n", crc, navdata->chksum, abs(crc-navdata->chksum));
        //navdata_getHeight();
      }
      navdata_CropBuffer(60);
    }
    else
    {
      // find start byte, copy all data from startbyte to buffer origin, update bytesread
      uint8_t * pint;
      pint = memchr(port->buffer, NAVDATA_START_BYTE, port->bytesRead);

      if (pint != NULL) {
        navdata_CropBuffer(pint - port->buffer);
      } else {
        // if the start byte was not found, it means there is junk in the buffer
        port->bytesRead = 0;
      }
    }
  }
}

void navdata_CropBuffer(int cropsize)
{
  if (port->bytesRead - cropsize < 0) {
    // TODO think about why the amount of bytes read minus the cropsize gets below zero
    printf("BytesRead(=%d) - Cropsize(=%d) may not be below zero. port->buffer=%p\n", port->bytesRead, cropsize, port->buffer);
    return;
  }

  memmove(port->buffer, port->buffer+cropsize, NAVDATA_BUFFER_SIZE-cropsize);
  port->bytesRead -= cropsize;
}

int16_t navdata_getHeight() {

  if (navdata->ultrasound > 10000) {
    return previousUltrasoundHeight;
  }

  int16_t ultrasoundHeight = 0;

  ultrasoundHeight = (navdata->ultrasound - 880) / 26.553;

  previousUltrasoundHeight = ultrasoundHeight;

  return ultrasoundHeight;
}

// The checksum should be calculated here: we don't know the algorithm
uint16_t navdata_checksum() {
  navdata_cks = 0;
  navdata_cks += navdata->nu_trame;
  navdata_cks += navdata->ax;
  navdata_cks += navdata->ay;
  navdata_cks += navdata->az;
  navdata_cks += navdata->vx;
  navdata_cks += navdata->vy;
  navdata_cks += navdata->vz;
  navdata_cks += navdata->temperature_acc;
  navdata_cks += navdata->temperature_gyro;
  navdata_cks += navdata->ultrasound;
  navdata_cks += navdata->us_debut_echo;
  navdata_cks += navdata->us_fin_echo;
  navdata_cks += navdata->us_association_echo;
  navdata_cks += navdata->us_distance_echo;
  navdata_cks += navdata->us_curve_time;
  navdata_cks += navdata->us_curve_value;
  navdata_cks += navdata->us_curve_ref;
  navdata_cks += navdata->nb_echo;
  navdata_cks += navdata->sum_echo;
  navdata_cks += navdata->gradient;
  navdata_cks += navdata->flag_echo_ini;
  navdata_cks += navdata->pressure;
  navdata_cks += navdata->temperature_pressure;
  navdata_cks += navdata->mx;
  navdata_cks += navdata->my;
  navdata_cks += navdata->mz;
//  navdata_cks += navdata->chksum;

  return 0; // we dont know how to calculate the checksum
}
