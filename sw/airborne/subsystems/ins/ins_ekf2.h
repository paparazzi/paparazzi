/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file subsystems/ins/ins_ekf2.h
 *
 * INS based in the EKF2 of PX4
 *
 */

#ifndef INS_EKF2_H
#define INS_EKF2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "subsystems/ahrs.h"
#include "subsystems/ins.h"

extern void ins_ekf2_init(void);
extern void ins_ekf2_update(void);

#ifdef __cplusplus
}
#endif

#endif /* INS_EKF2_H */
