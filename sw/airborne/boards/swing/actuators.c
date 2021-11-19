/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file boards/swing/actuators.c
 * Actuator driver for the swing
 */


#include <sys/ioctl.h>
#include <fcntl.h>

/* build ioctl unique identifiers for R/W operations */
#define PWM_MAGIC 'p'
typedef struct { unsigned int val[4]; } __attribute__ ((packed)) pwm_delos_quadruplet;
#define PWM_DELOS_SET_RATIOS _IOR(PWM_MAGIC, 9,  pwm_delos_quadruplet*)
#define PWM_DELOS_SET_SPEEDS _IOR(PWM_MAGIC, 10, pwm_delos_quadruplet*)
#define PWM_DELOS_SET_CTRL   _IOR(PWM_MAGIC, 11, unsigned int)
#define PWM_DELOS_REQUEST    _IO(PWM_MAGIC, 12)

#define PWM_NB_BITS   (9)

/* PWM can take value between 0 and 511 */
#ifndef PWM_TOTAL_RANGE
#define PWM_TOTAL_RANGE (1<<PWM_NB_BITS)
#endif

#define PWM_REG_RATIO_PRECISION_MASK (PWM_NB_BITS<<16)
#define PWM_REG_SATURATION (PWM_REG_RATIO_PRECISION_MASK|PWM_TOTAL_RANGE)

#include "modules/actuators/actuators.h"
#include "modules/actuators/motor_mixing.h"
#include "actuators.h"
#include "autopilot.h"

struct ActuatorsSwing actuators_swing;
static int actuators_fd;

// Start/stop PWM
enum {
  SiP6_PWM0_START = (1<<0),
  SiP6_PWM1_START = (1<<1),
  SiP6_PWM2_START = (1<<2),
  SiP6_PWM3_START = (1<<3),
};

void actuators_swing_init(void)
{
  actuators_fd = open("/dev/pwm", O_RDWR);

  pwm_delos_quadruplet m = {{ 1, 1, 1, 1 }};
  int ret __attribute__((unused)) = ioctl(actuators_fd, PWM_DELOS_SET_SPEEDS, &m);
#if ACTUATORS_SWING_DEBUG
  printf("Return Speeds: %d\n", ret);
#endif

  actuators_swing_commit();

  unsigned int control_reg = (SiP6_PWM0_START|SiP6_PWM1_START|SiP6_PWM2_START|SiP6_PWM3_START);

  ret = ioctl(actuators_fd, PWM_DELOS_SET_CTRL, &control_reg);
#if ACTUATORS_SWING_DEBUG
  printf("Return control: %d\n", ret);
#endif
}

void actuators_swing_commit(void)
{
  pwm_delos_quadruplet m;

  m.val[0] = actuators_swing.rpm_ref[0] & 0xffff;
  m.val[1] = actuators_swing.rpm_ref[1] & 0xffff;
  m.val[2] = actuators_swing.rpm_ref[2] & 0xffff;
  m.val[3] = actuators_swing.rpm_ref[3] & 0xffff;


  if( actuators_swing.rpm_ref[0] > (PWM_TOTAL_RANGE) ) { m.val[0] = PWM_REG_SATURATION; }
  if( actuators_swing.rpm_ref[1] > (PWM_TOTAL_RANGE) ) { m.val[1] = PWM_REG_SATURATION; }
  if( actuators_swing.rpm_ref[2] > (PWM_TOTAL_RANGE) ) { m.val[2] = PWM_REG_SATURATION; }
  if( actuators_swing.rpm_ref[3] > (PWM_TOTAL_RANGE) ) { m.val[3] = PWM_REG_SATURATION; }

  if( actuators_swing.rpm_ref[0] < 0 ) { m.val[0] = 0; }
  if( actuators_swing.rpm_ref[1] < 0 ) { m.val[1] = 0; }
  if( actuators_swing.rpm_ref[2] < 0 ) { m.val[2] = 0; }
  if( actuators_swing.rpm_ref[3] < 0 ) { m.val[3] = 0; }

  /* The upper 16-bit word of the ratio register contains the number
   * of bits used to code the ratio command  */
  m.val[0] |= PWM_REG_RATIO_PRECISION_MASK;
  m.val[1] |= PWM_REG_RATIO_PRECISION_MASK;
  m.val[2] |= PWM_REG_RATIO_PRECISION_MASK;
  m.val[3] |= PWM_REG_RATIO_PRECISION_MASK;

  int ret __attribute__((unused)) = ioctl(actuators_fd, PWM_DELOS_SET_RATIOS, &m);

#if ACTUATORS_SWING_DEBUG
  RunOnceEvery(512, printf("Return ratios: %d (ratios: %d %d %d %d, pwm: %d %d %d %d\n",
       ret,
       m.val[0], m.val[1], m.val[2], m.val[3],
       actuators_swing.rpm_ref[0],
       actuators_swing.rpm_ref[1],
       actuators_swing.rpm_ref[2],
       actuators_swing.rpm_ref[3])
      );
#endif
}

