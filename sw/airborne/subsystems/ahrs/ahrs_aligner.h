/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 */

/**
 * @file subsystems/ahrs/ahrs_aligner.h
 *
 * Interface to align the AHRS via low-passed measurements at startup.
 *
 */

#ifndef AHRS_ALIGNER_H
#define AHRS_ALIGNER_H

#include "std.h"
#include "math/pprz_algebra_int.h"

#define AHRS_ALIGNER_UNINIT  0
#define AHRS_ALIGNER_RUNNING 1
#define AHRS_ALIGNER_LOCKED  2

struct AhrsAligner {
  struct Int32Rates lp_gyro;
  struct Int32Vect3 lp_accel;
  struct Int32Vect3 lp_mag;
  int32_t           noise;
  int32_t           low_noise_cnt;
  uint8_t           status;
};

extern struct AhrsAligner ahrs_aligner;

extern void ahrs_aligner_init(void);
extern void ahrs_aligner_restart(void);
extern void ahrs_aligner_run(void);

#endif /* AHRS_ALIGNER_H */
