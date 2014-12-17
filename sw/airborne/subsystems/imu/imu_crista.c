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

#include "subsystems/imu.h"

volatile bool_t ADS8344_available;
uint16_t ADS8344_values[ADS8344_NB_CHANNELS];

void imu_impl_init(void)
{

  ADS8344_available = FALSE;

  imu_crista_arch_init();

#ifdef USE_AMI601
  ami601_init();
#endif
#ifdef USE_HMC5843
  hmc5843_init();
#endif

}

void imu_periodic(void)
{

  ImuCristaArchPeriodic();
#ifdef USE_AMI601
  RunOnceEvery(10, { ami601_read(); });
#endif
#ifdef USE_HMC5843
  hmc5843_periodic();
#endif
}
