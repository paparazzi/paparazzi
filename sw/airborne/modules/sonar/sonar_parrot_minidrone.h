/*
 * Copyright (C) 2017 Paparazzi Team
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

/** @file modules/sonar/sonar_parrot_minidrone.h
 *  @brief Driver for a Parrot Minidrone to be able to read build in ultrasonic ranging sensor
 */

#ifndef SONAR_PARROT_MINIDRONE_H
#define SONAR_PARROT_MINIDRONE_H

#include <stdint.h>

// To be parameter compatible with the Bebop Sonar we added this struct
struct SonarParrotMinidrone {
  uint16_t meas;          ///< Raw ADC value
  uint16_t offset;        ///< Sonar offset in ADC units
  float distance;         ///< Distance measured in meters
};

extern struct SonarParrotMinidrone sonar_parrot_minidrone;

extern void sonar_parrot_minidrone_init(void);
extern void sonar_parrot_minidrone_downlink(void);//Handy for debugging

#endif /* SONAR_PARROT_MINIDRONE_H */
