/*
 * Copyright (C) 2026 Tomaso De Ponti <tomaso@aerogriduav.com>
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
 * @file modules/sound/buzzer_tunes.h
 * @brief Play tunes on a passive piezo buzzer (e.g. a startup tune)
 */

#ifndef BUZZER_TUNES_H
#define BUZZER_TUNES_H

#include "std.h"

/** Tunes, order must match the values of the tune setting */
enum buzzer_tunes_t {
  BUZZER_TUNES_NONE,
  BUZZER_TUNES_STARTUP,
  BUZZER_TUNES_ODE_TO_JOY,
  BUZZER_TUNES_TWINKLE,
  BUZZER_TUNES_ANTHEM_ITALY,
  BUZZER_TUNES_ANTHEM_NETHERLANDS,
  BUZZER_TUNES_ANTHEM_FRANCE,
  BUZZER_TUNES_HAPPY_BIRTHDAY,
  BUZZER_TUNES_MUSHROOM,
  BUZZER_TUNES_GRAN_VALS,
  BUZZER_TUNES_FUR_ELISE,
  BUZZER_TUNES_LA_CUCARACHA,
  BUZZER_TUNES_WILLIAM_TELL,
  BUZZER_TUNES_CUSTOM_1,    ///< custom RTTTL tunes, see BUZZER_TUNES_RTTTL_x
  BUZZER_TUNES_CUSTOM_2,
  BUZZER_TUNES_CUSTOM_3,
  BUZZER_TUNES_NB
};

extern uint8_t buzzer_tunes_tune; ///< tune being played, BUZZER_TUNES_NONE when silent

extern void buzzer_tunes_init(void);
extern void buzzer_tunes_periodic(void);

/** Start a tune (BUZZER_TUNES_NONE stops), also the settings handler */
extern void buzzer_tunes_play(uint8_t tune);

#endif /* BUZZER_TUNES_H */
