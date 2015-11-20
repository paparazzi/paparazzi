/*
 * Copyright (C) 2015 The Paparazzi Team
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
 * @file firmwares/rotorcraft/main_fbw.h
 *
 * Fly By Wire:
 *
 * Reads Radiocontrol
 * Reads Intermcu
 * Sets Actuators
 *
 * if no rc but autopilot then RC_LOST_FBW_MODE (define below)
 * if no rc while in auto mode then RC_LOST_IN_AUTO_FBW_MODE (define below)
 * if no ap but rc then AP_LOST_FBW_MODE (define below)
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef SITL
#define STATIC_INLINE extern
#else
#define STATIC_INLINE static inline
#endif

/** mode to enter when RC is lost while using a mode with RC input */
#ifndef RC_LOST_FBW_MODE
#define RC_LOST_FBW_MODE FBW_MODE_FAILSAFE
#endif

/** mode to enter when AP is lost while using autopilot */
#ifndef RC_LOST_IN_AUTO_FBW_MODE
#define RC_LOST_IN_AUTO_FBW_MODE FBW_MODE_AUTO
#endif

/** mode to enter when AP is lost while using autopilot */
#ifndef AP_LOST_FBW_MODE
#define AP_LOST_FBW_MODE FBW_MODE_FAILSAFE
#endif



/** Switching between FBW and autopilot is done with RADIO_FBW_MODE: default is to re-use RADIO_MODE */

#ifndef RADIO_FBW_MODE
#define RADIO_FBW_MODE RADIO_MODE
#endif

STATIC_INLINE void main_init(void);
STATIC_INLINE void main_event(void);
STATIC_INLINE void handle_periodic_tasks(void);
STATIC_INLINE void main_periodic(void);
STATIC_INLINE void telemetry_periodic(void);


#endif /* MAIN_H */
