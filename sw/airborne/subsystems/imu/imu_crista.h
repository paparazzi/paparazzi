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

#ifndef IMU_CRISTA_H
#define IMU_CRISTA_H

#include "subsystems/imu.h"
#include "generated/airframe.h"

#define ADS8344_NB_CHANNELS 8
extern uint16_t ADS8344_values[ADS8344_NB_CHANNELS];
extern volatile bool_t ADS8344_available;

/* underlying architecture */
#include "subsystems/imu/imu_crista_arch.h"
/* must be defined by underlying architecture */
extern void imu_crista_arch_init(void);

extern void imu_christa_event(void);

#define ImuEvent imu_christa_event

#endif /* IMU_CRISTA_H */
