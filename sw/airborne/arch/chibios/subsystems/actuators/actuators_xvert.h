/*
 * Copyright (C) Kevin van Hecke
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file actuators_xvert.h
 * @author Kevin van Hecke
 *  Actuators driver for X-vert VTOL motor controllers. Contains two normal pwm servos, and two custom driven escs through a propriety uart protocol.
 */

#ifndef ACTUATORS_XVERT_H
#define ACTUATORS_XVERT_H

#include "subsystems/actuators/actuators_pwm_arch.h"

#define ESCS_START_BYTE 0xFE
#define ESCS_DATA_FLIPBIT 16384
#define ESCS_DATA_MYSTERYBIT 32768

struct EscData {
    unsigned char start; //0xfe
    unsigned char len; //8
    unsigned char id; //2

    //1200 - 1800, maybe 1100-1900
    //1160 = off, max ~1880
    //in both data ints, 2e byte toggles the 64 bit  (15e bit in total). If on, its the first, if off its the other
    //in d2 16e bit always on

    uint32_t d1 ;
    uint32_t d2 ;
    unsigned char crc;
}__attribute__((__packed__));


extern int32_t actuators_xvert_values[ACTUATORS_PWM_NB];
extern void actuators_xvert_commit(void);
extern void actuators_xvert_init(void);

#define ActuatorsXvertInit  actuators_xvert_init
#define ActuatorXvertSet(_i, _v) { actuators_xvert_values[_i] = _v; }
#define ActuatorsXvertCommit  actuators_xvert_commit



#endif
