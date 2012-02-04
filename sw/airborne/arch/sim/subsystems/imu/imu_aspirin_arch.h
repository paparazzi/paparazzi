/*
 * $Id$
 *
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

/*
 *
 * simulator plug for the booz2 v1 imu arch dependant functions
 *
 */
#ifndef IMU_ASPIRIN_ARCH_H
#define IMU_ASPIRIN_ARCH_H


extern void imu_feed_gyro_accel(void);
extern void imu_feed_mag(void);

#define imu_aspirin_arch_int_enable() {}
#define imu_aspirin_arch_int_disable() {}
#define adxl345_write_to_reg(foo, bar) {}
#define adxl345_clear_rx_buf() {}
#define adxl345_start_reading_data() {}

static inline int imu_aspirin_eoc(void)
{
  return 1;
}


#endif /* IMU_ASPIRIN_ARCH_H */
