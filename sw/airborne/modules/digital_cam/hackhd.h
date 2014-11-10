/*
 * Copyright (C) 2014 Gautier Hattenberger
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


/** @file modules/digital_cam/hackhd.h
 *  @brief Digital video/photo recorder HackHD control
 *
 * Provides the control of the HackHD power, start and stop of recording.
 * If you are using firmware >= 1.1.5, it is also possible to take pictures
 * according to the parameter in the config.txt file (on HackHD SD card).
 * It is not possible to have both video and photo at the same time.
 * This driver starts the HackHD in standby mode and trigger the start/stop
 * of recording or take a picture.
 * Minimum time between two pictures is 2 seconds.
 *
 * It is mandatory to configure the control GPIO:
 * @verbatim
 *   <configure name="HACKHD_GPIO" value="GPIOC,GPIO5"/>
 * @endverbatim
 *
 */

#ifndef HACKHD_H
#define HACKHD_H

#include "std.h"
#include "math/pprz_geodetic_float.h"

enum hackhd_status {
  HACKHD_NONE,
  HACKHD_POWER_ON,
  HACKHD_POWER_OFF,
  HACKHD_START_RECORD,
  HACKHD_STOP_RECORD,
  HACKHD_SHOOT,
  HACKHD_AUTOSHOOT_START
};

struct HackHD {
  enum hackhd_status status;
  uint32_t timer;
  int16_t photo_nr;
  uint32_t autoshoot;
  struct EnuCoor_f last_shot_pos;
  uint32_t log_delay;
};

extern struct HackHD hackhd;

extern void hackhd_init(void);
extern void hackhd_periodic(void);
extern void hackhd_autoshoot(void);
extern void hackhd_autoshoot_start(void);
extern void hackhd_command(enum hackhd_status cmd);

// macro for setting handler
#define hackhd_SendCmd(cmd) hackhd_command(cmd)

#endif // HACKHD_H
