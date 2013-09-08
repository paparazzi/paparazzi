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

#include <stdint.h>

typedef struct
{
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

} __attribute__ ((packed)) measures_t;

struct bmp180_baro_calibration
{
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

extern measures_t navdata;
struct bmp180_baro_calibration baro_calibration;
uint16_t navdata_cks;
uint8_t navdata_imu_available;
uint8_t navdata_baro_available;
int16_t previousUltrasoundHeight;
uint8_t baro_calibrated;

int navdata_init(void);
void navdata_close(void);

void navdata_read(void);
void navdata_update(void);
void navdata_CropBuffer(int cropsize);

uint16_t navdata_checksum(void);
int16_t navdata_getHeight(void);

void acquire_baro_calibration(void);

#endif /* NAVDATA_H_ */
