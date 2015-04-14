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
 * @file boards/ardrone/navdata.h
 * ardrone2 navdata aquisition driver.
 *
 * The ardrone2 provides a navdata stream of packets
 * containing info about all sensors at a rate of 200Hz.
 */

#ifndef NAVDATA_H_
#define NAVDATA_H_

#include "std.h"

/**
 * Main navdata structure from the navdata board
 * This is received from the navdata board at ~200Hz
 */
struct navdata_measure_t {
  uint16_t taille;
  uint16_t nu_trame;

  uint16_t ax;
  uint16_t ay;
  uint16_t az;

  int16_t vx;
  int16_t vy;
  int16_t vz;
  uint16_t temperature_acc;
  uint16_t temperature_gyro;

  uint16_t ultrasound;

  uint16_t us_debut_echo;
  uint16_t us_fin_echo;
  uint16_t us_association_echo;
  uint16_t us_distance_echo;

  uint16_t us_curve_time;
  uint16_t us_curve_value;
  uint16_t us_curve_ref;

  uint16_t nb_echo;

  uint32_t sum_echo; //unsigned long
  int16_t gradient;

  uint16_t flag_echo_ini;

  int32_t pressure;
  uint16_t temperature_pressure;

  int16_t my;
  int16_t mx;
  int16_t mz;

  uint16_t chksum;

} __attribute__((packed));

/* The baro calibration received from the navboard */
struct bmp180_calib_t {
  int16_t ac1;
  int16_t ac2;
  int16_t ac3;
  uint16_t ac4;
  uint16_t ac5;
  uint16_t ac6;
  int16_t b1;
  int16_t b2;
  int16_t mb;
  int16_t mc;
  int16_t md;

  // These values are calculated
  int32_t b5;
};

/* Navdata board defines */
#define NAVDATA_PACKET_SIZE       60
#define NAVDATA_START_BYTE        0x3A
#define NAVDATA_CMD_START         0x01
#define NAVDATA_CMD_STOP          0x02
#define NAVDATA_CMD_BARO_CALIB    0x17

#define ARDRONE_GPIO_PORT         0x32524
#define ARDRONE_GPIO_PIN_NAVDATA  177

/* Main navdata structure */
struct navdata_t {
  bool_t is_initialized;                  ///< Check if the navdata board is initialized
  int fd;                                 ///< The navdata file pointer

  uint32_t totalBytesRead;
  uint32_t packetsRead;
  uint32_t checksum_errors;
  uint32_t lost_imu_frames;
  uint16_t last_packet_number;

  struct navdata_measure_t measure;       ///< Main navdata packet receieved from navboard
  struct bmp180_calib_t bmp180_calib;     ///< BMP180 calibration receieved from navboard

  bool_t baro_calibrated;                 ///< Whenever the baro is calibrated
  bool_t baro_available;                  ///< Whenever the baro is available
  bool_t imu_lost;                        ///< Whenever the imu is lost
};
extern struct navdata_t navdata;


bool_t navdata_init(void);
void navdata_update(void);
int16_t navdata_height(void);

/* This should be moved to the uart handling part! */
ssize_t full_write(int fd, const uint8_t *buf, size_t count);
ssize_t full_read(int fd, uint8_t *buf, size_t count);

#endif /* NAVDATA_H_ */
