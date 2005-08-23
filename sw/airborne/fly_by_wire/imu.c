/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2005 Pascal Brisset, Antoine Drouin
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

#include "airframe.h"

#if defined(SECTION_IMU_3DMG) && defined(SECTION_IMU_ANALOG)
#error "IMU_3DMG and IMU_ANALOG cannot be defined simultaneously"
#endif

#include "imu.h"

int16_t roll_dot, pitch_dot, yaw_dot;

#ifdef SECTION_IMU_3DMG
#warning "Compiling imu.c for 3DMG"
#include "3dmg.h"
int16_t roll, pitch, yaw;

void imu_init ( void ) {
  _3dmg_set_continuous_mode();
}

void imu_update ( void ) {
  roll_dot = _3dmg_roll_dot;
  pitch_dot = _3dmg_pitch_dot;
  yaw_dot = _3dmg_yaw_dot;

  roll = _3dmg_roll;
  pitch = _3dmg_pitch;
  yaw = _3dmg_yaw;
}

void imu_capture_neutral ( void ) {
  _3dmg_capture_neutral();
}

#endif // 3DMG

#ifdef SECTION_IMU_ANALOG
#warning "Compiling imu.c for ANALOG"
#include "adc_fbw.h"

struct adc_buf buf_roll_dot;
struct adc_buf buf_pitch_dot;
struct adc_buf buf_yaw_dot;

uint16_t raw_roll_dot  = 512*AV_NB_SAMPLE;
uint16_t raw_pitch_dot = 512*AV_NB_SAMPLE;
uint16_t raw_yaw_dot   = 512*AV_NB_SAMPLE;

uint16_t raw_roll_dot_neutral  = 512*AV_NB_SAMPLE;
uint16_t raw_pitch_dot_neutral = 512*AV_NB_SAMPLE;
uint16_t raw_yaw_dot_neutral   = 512*AV_NB_SAMPLE;

void imu_init ( void ) {
  adc_buf_channel(IMU_ADC_ROLL_DOT, &buf_roll_dot);
  adc_buf_channel(IMU_ADC_PITCH_DOT, &buf_pitch_dot);
  adc_buf_channel(IMU_ADC_YAW_DOT, &buf_yaw_dot);
}

void imu_update ( void ) {
  raw_roll_dot = buf_roll_dot.sum;
  raw_pitch_dot = buf_pitch_dot.sum;
  raw_yaw_dot = buf_yaw_dot.sum;
  roll_dot = (int16_t)(raw_roll_dot - raw_roll_dot_neutral) / AV_NB_SAMPLE;
  pitch_dot = (int16_t)(raw_pitch_dot - raw_pitch_dot_neutral) / AV_NB_SAMPLE;
  yaw_dot = (int16_t)(raw_yaw_dot - raw_yaw_dot_neutral) / AV_NB_SAMPLE;;
}

void imu_capture_neutral ( void ) {
  raw_roll_dot_neutral = raw_roll_dot;
  raw_pitch_dot_neutral = raw_pitch_dot;
  raw_yaw_dot_neutral = raw_yaw_dot;
}

#endif // ANALOG

