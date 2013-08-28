/*
 * Copyright (C) 2011 Gautier Hattenberger
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
 * @file modules/sensors/baro_board_module.h
 *
 * Wrapper for the board specific barometer.
 */

#ifndef BARO_BOARD_MODULE_H
#define BARO_BOARD_MODULE_H

#include "subsystems/sensors/baro.h"

/** Absolute baro macro mapping.
 *  Select the baro module you want to use to feed the common baro interface
 *  in your airframe file when configuring baro_board module
 *  ex:
 *   for module baro_ets
 * @verbatim
 *   <define name="BARO_ABS_EVENT" value="BaroEtsUpdate"/>
 * @endverbatim
 */
#ifndef BARO_ABS_EVENT
#define BARO_ABS_EVENT NoBaro
#endif

/** Differential baro macro mapping.
 *  TODO
 */
#ifndef BARO_DIFF_EVENT
#define BARO_DIFF_EVENT NoBaro
#endif

#define NoBaro(_b, _h) {}

/** BaroEvent macro.
 *  Need to be maped to one the external baro running has a module
 *
 *  Undef if necessary (already defined in a baro_board.h file)
 */
#ifdef BaroEvent
#undef BaroEvent
#endif

#define BaroEvent(_b_abs_handler, _b_diff_handler) {  \
    BARO_ABS_EVENT(baro.absolute, _b_abs_handler);       \
    BARO_DIFF_EVENT(baro.differential, _b_diff_handler); \
}


#endif
