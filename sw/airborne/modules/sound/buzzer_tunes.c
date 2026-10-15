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
 * @file modules/sound/buzzer_tunes.c
 * @brief Play tunes on a passive piezo buzzer (e.g. a startup tune)
 */

#include "modules/sound/buzzer_tunes.h"
#include "modules/sound/buzzer_arch.h"
#include "mcu_periph/sys_time.h"
#include "generated/airframe.h"

/** Tune played at startup, BUZZER_TUNES_NONE for silence */
#ifndef BUZZER_TUNES_STARTUP_TUNE
#define BUZZER_TUNES_STARTUP_TUNE BUZZER_TUNES_STARTUP
#endif

/** Silence at the end of each note, so repeated notes are heard separately [ms] */
#define BUZZER_TUNES_GAP_MS 30

/** Note frequencies [Hz], s for sharp, octaves 5 to 7 sound loudest on a piezo */
#define REST 0
#define C5 523
#define Cs5 554
#define D5 587
#define Ds5 622
#define E5 659
#define F5 698
#define Fs5 740
#define G5 784
#define Gs5 831
#define A5 880
#define As5 932
#define B5 988
#define C6 1047
#define Cs6 1109
#define D6 1175
#define Ds6 1245
#define E6 1319
#define F6 1397
#define Fs6 1480
#define G6 1568
#define Gs6 1661
#define A6 1760
#define As6 1865
#define B6 1976
#define C7 2093
#define Cs7 2217
#define D7 2349
#define Ds7 2489
#define E7 2637
#define F7 2794
#define Fs7 2960
#define G7 3136
#define Gs7 3322
#define A7 3520
#define As7 3729
#define B7 3951

/** Duration of a number of beats at a tempo [ms] */
#define BEATS(_b, _bpm) ((uint16_t)((_b) * 60000.0 / (_bpm)))

struct buzzer_note {
  uint16_t freq;  ///< [Hz], REST for silence
  uint16_t ms;    ///< duration [ms]
};

#define TUNE_END {0, 0}

/** Custom tunes (Custom 1..3) in RTTTL format ("name:d=4,o=6,b=120:c,e,g"), set in the airframe, empty for none */
#ifndef BUZZER_TUNES_RTTTL_1
#define BUZZER_TUNES_RTTTL_1 ""
#endif
#ifndef BUZZER_TUNES_RTTTL_2
#define BUZZER_TUNES_RTTTL_2 ""
#endif
#ifndef BUZZER_TUNES_RTTTL_3
#define BUZZER_TUNES_RTTTL_3 ""
#endif

/** Short rising arpeggio */
static const struct buzzer_note tune_startup[] = {
  {C6, 120}, {E6, 120}, {G6, 120}, {C7, 360}, TUNE_END
};

/** Beethoven, Ode to Joy, full theme (public domain) */
static const struct buzzer_note tune_ode_to_joy[] = {
  {E6, 300}, {E6, 300}, {F6, 300}, {G6, 300}, {G6, 300}, {F6, 300}, {E6, 300}, {D6, 300},
  {C6, 300}, {C6, 300}, {D6, 300}, {E6, 300}, {E6, 450}, {D6, 150}, {D6, 600},
  {E6, 300}, {E6, 300}, {F6, 300}, {G6, 300}, {G6, 300}, {F6, 300}, {E6, 300}, {D6, 300},
  {C6, 300}, {C6, 300}, {D6, 300}, {E6, 300}, {D6, 450}, {C6, 150}, {C6, 600},
  {D6, 300}, {D6, 300}, {E6, 300}, {C6, 300}, {D6, 300}, {E6, 150}, {F6, 150}, {E6, 300}, {C6, 300},
  {D6, 300}, {E6, 150}, {F6, 150}, {E6, 300}, {D6, 300}, {C6, 300}, {D6, 300}, {G5, 600},
  {E6, 300}, {E6, 300}, {F6, 300}, {G6, 300}, {G6, 300}, {F6, 300}, {E6, 300}, {D6, 300},
  {C6, 300}, {C6, 300}, {D6, 300}, {E6, 300}, {D6, 450}, {C6, 150}, {C6, 600}, TUNE_END
};

/** Twinkle Twinkle Little Star, full song (public domain) */
static const struct buzzer_note tune_twinkle[] = {
  {C6, 300}, {C6, 300}, {G6, 300}, {G6, 300}, {A6, 300}, {A6, 300}, {G6, 600},
  {F6, 300}, {F6, 300}, {E6, 300}, {E6, 300}, {D6, 300}, {D6, 300}, {C6, 600},
  {G6, 300}, {G6, 300}, {F6, 300}, {F6, 300}, {E6, 300}, {E6, 300}, {D6, 600},
  {G6, 300}, {G6, 300}, {F6, 300}, {F6, 300}, {E6, 300}, {E6, 300}, {D6, 600},
  {C6, 300}, {C6, 300}, {G6, 300}, {G6, 300}, {A6, 300}, {A6, 300}, {G6, 600},
  {F6, 300}, {F6, 300}, {E6, 300}, {E6, 300}, {D6, 300}, {D6, 300}, {C6, 600}, TUNE_END
};

/** Italy, Il Canto degli Italiani, first verse (Novaro 1847, public domain) */
#define IT_BPM 112
static const struct buzzer_note tune_anthem_italy[] = {
  {F5, BEATS(1, IT_BPM)}, {F5, BEATS(0.75, IT_BPM)}, {G5, BEATS(0.25, IT_BPM)}, {F5, BEATS(1, IT_BPM)},
  {REST, BEATS(1, IT_BPM)}, {D6, BEATS(1, IT_BPM)}, {D6, BEATS(0.75, IT_BPM)}, {Ds6, BEATS(0.25, IT_BPM)},
  {D6, BEATS(1, IT_BPM)}, {REST, BEATS(1, IT_BPM)}, {D6, BEATS(1, IT_BPM)}, {F6, BEATS(0.75, IT_BPM)},
  {Ds6, BEATS(0.25, IT_BPM)}, {D6, BEATS(2, IT_BPM)}, {C6, BEATS(1, IT_BPM)}, {D6, BEATS(0.75, IT_BPM)},
  {C6, BEATS(0.25, IT_BPM)}, {As5, BEATS(1, IT_BPM)}, {REST, BEATS(1, IT_BPM)}, {D6, BEATS(1, IT_BPM)},
  {D6, BEATS(1, IT_BPM)}, {A5, BEATS(2, IT_BPM)}, {As5, BEATS(0.75, IT_BPM)}, {C6, BEATS(0.25, IT_BPM)},
  {As5, BEATS(0.75, IT_BPM)}, {A5, BEATS(0.25, IT_BPM)}, {G5, BEATS(1, IT_BPM)}, {REST, BEATS(1, IT_BPM)},
  {As5, BEATS(1, IT_BPM)}, {A5, BEATS(0.75, IT_BPM)}, {As5, BEATS(0.25, IT_BPM)}, {C6, BEATS(1, IT_BPM)},
  {REST, BEATS(1, IT_BPM)}, {D5, BEATS(1, IT_BPM)}, {D6, BEATS(2, IT_BPM)}, {Ds6, BEATS(1, IT_BPM)},
  {F5, BEATS(1, IT_BPM)}, {F5, BEATS(0.75, IT_BPM)}, {G5, BEATS(0.25, IT_BPM)}, {F5, BEATS(0.5, IT_BPM)},
  {REST, BEATS(1.5, IT_BPM)}, {D6, BEATS(1, IT_BPM)}, {D6, BEATS(0.75, IT_BPM)}, {Ds6, BEATS(0.25, IT_BPM)},
  {D6, BEATS(2, IT_BPM)}, {D6, BEATS(1, IT_BPM)}, {F6, BEATS(0.75, IT_BPM)}, {Ds6, BEATS(0.25, IT_BPM)},
  {D6, BEATS(1.5, IT_BPM)}, {F6, BEATS(0.5, IT_BPM)}, {C6, BEATS(0.5, IT_BPM)}, {F6, BEATS(0.5, IT_BPM)},
  {As5, BEATS(2, IT_BPM)}, TUNE_END
};

/** Netherlands, Wilhelmus, first verse (16th century, public domain) */
#define NL_BPM 90
static const struct buzzer_note tune_anthem_netherlands[] = {
  {D5, BEATS(1, NL_BPM)}, {G5, BEATS(1, NL_BPM)}, {G5, BEATS(1, NL_BPM)}, {A5, BEATS(0.5, NL_BPM)},
  {B5, BEATS(0.5, NL_BPM)}, {C6, BEATS(0.5, NL_BPM)}, {A5, BEATS(0.5, NL_BPM)}, {B5, BEATS(1, NL_BPM)},
  {A5, BEATS(0.5, NL_BPM)}, {B5, BEATS(0.5, NL_BPM)}, {C6, BEATS(1, NL_BPM)}, {B5, BEATS(1, NL_BPM)},
  {A5, BEATS(0.5, NL_BPM)}, {G5, BEATS(0.5, NL_BPM)}, {A5, BEATS(1, NL_BPM)}, {G5, BEATS(3, NL_BPM)},
  {B5, BEATS(0.5, NL_BPM)}, {C6, BEATS(0.5, NL_BPM)}, {D6, BEATS(2, NL_BPM)}, {E6, BEATS(1, NL_BPM)},
  {D6, BEATS(2, NL_BPM)}, {C6, BEATS(1, NL_BPM)}, {B5, BEATS(1, NL_BPM)}, {A5, BEATS(0.5, NL_BPM)},
  {B5, BEATS(0.5, NL_BPM)}, {C6, BEATS(1, NL_BPM)}, {B5, BEATS(1, NL_BPM)}, {A5, BEATS(1, NL_BPM)},
  {G5, BEATS(1, NL_BPM)}, {A5, BEATS(3, NL_BPM)}, {D5, BEATS(1, NL_BPM)}, {G5, BEATS(0.5, NL_BPM)},
  {Fs5, BEATS(0.5, NL_BPM)}, {G5, BEATS(0.5, NL_BPM)}, {A5, BEATS(0.5, NL_BPM)}, {B5, BEATS(1, NL_BPM)},
  {A5, BEATS(2, NL_BPM)}, {G5, BEATS(1, NL_BPM)}, {Fs5, BEATS(1, NL_BPM)}, {D5, BEATS(1, NL_BPM)},
  {E5, BEATS(0.5, NL_BPM)}, {Fs5, BEATS(0.5, NL_BPM)}, {G5, BEATS(1, NL_BPM)}, {G5, BEATS(1, NL_BPM)},
  {Fs5, BEATS(1, NL_BPM)}, {G5, BEATS(3, NL_BPM)}, TUNE_END
};

/** France, La Marseillaise, opening to "l'étendard sanglant est levé" (Rouget de Lisle 1792, public domain) */
#define FR_BPM 105
static const struct buzzer_note tune_anthem_france[] = {
  {D5, BEATS(0.25, FR_BPM)}, {D5, BEATS(0.75, FR_BPM)}, {D5, BEATS(0.25, FR_BPM)}, {G5, BEATS(1, FR_BPM)},
  {G5, BEATS(1, FR_BPM)}, {A5, BEATS(1, FR_BPM)}, {A5, BEATS(1, FR_BPM)}, {D6, BEATS(1.5, FR_BPM)},
  {B5, BEATS(0.5, FR_BPM)}, {G5, BEATS(0.75, FR_BPM)}, {G5, BEATS(0.25, FR_BPM)}, {B5, BEATS(0.75, FR_BPM)},
  {G5, BEATS(0.25, FR_BPM)}, {E5, BEATS(1, FR_BPM)}, {C6, BEATS(1, FR_BPM)}, {REST, BEATS(1, FR_BPM)},
  {A5, BEATS(0.75, FR_BPM)}, {Fs5, BEATS(0.25, FR_BPM)}, {G5, BEATS(1, FR_BPM)}, {REST, BEATS(2, FR_BPM)},
  {G5, BEATS(0.75, FR_BPM)}, {A5, BEATS(0.25, FR_BPM)}, {B5, BEATS(1, FR_BPM)}, {B5, BEATS(1, FR_BPM)},
  {B5, BEATS(1, FR_BPM)}, {C6, BEATS(0.75, FR_BPM)}, {B5, BEATS(0.25, FR_BPM)}, {B5, BEATS(1, FR_BPM)},
  {A5, BEATS(1, FR_BPM)}, {REST, BEATS(1, FR_BPM)}, {A5, BEATS(0.75, FR_BPM)}, {B5, BEATS(0.25, FR_BPM)},
  {C6, BEATS(1, FR_BPM)}, {C6, BEATS(1, FR_BPM)}, {C6, BEATS(1, FR_BPM)}, {D6, BEATS(0.75, FR_BPM)},
  {C6, BEATS(0.25, FR_BPM)}, {B5, BEATS(1, FR_BPM)}, {REST, BEATS(2, FR_BPM)}, {D6, BEATS(0.75, FR_BPM)},
  {D6, BEATS(0.25, FR_BPM)}, {D6, BEATS(1, FR_BPM)}, {B5, BEATS(0.75, FR_BPM)}, {G5, BEATS(0.25, FR_BPM)},
  {D6, BEATS(1, FR_BPM)}, {B5, BEATS(0.75, FR_BPM)}, {G5, BEATS(0.25, FR_BPM)}, {D5, BEATS(3, FR_BPM)}, TUNE_END
};

/** Happy Birthday (public domain) */
static const struct buzzer_note tune_happy_birthday[] = {
  {G5, 300}, {G5, 100}, {A5, 400}, {G5, 400}, {C6, 400}, {B5, 800},
  {G5, 300}, {G5, 100}, {A5, 400}, {G5, 400}, {D6, 400}, {C6, 800},
  {G5, 300}, {G5, 100}, {G6, 400}, {E6, 400}, {C6, 400}, {B5, 400}, {A5, 800},
  {F6, 300}, {F6, 100}, {E6, 400}, {C6, 400}, {D6, 400}, {C6, 800}, TUNE_END
};


/** Mushroom Song */
static const struct buzzer_note tune_mushroom[] = {
  {E7, 150}, {E7, 150}, {REST, 150}, {E7, 150}, {REST, 150}, {C7, 150}, {E7, 150}, {REST, 150},
  {G7, 300}, {REST, 300}, {G6, 300}, {REST, 300}, {C7, 450}, {G6, 150}, {REST, 300}, {E6, 450},
  {A6, 300}, {B6, 300}, {As6, 150}, {A6, 300}, {G6, 200}, {E7, 200}, {G7, 200}, {A7, 300},
  {F7, 150}, {G7, 150}, {REST, 150}, {E7, 300}, {C7, 150}, {D7, 150}, {B6, 450}, {C7, 450},
  {G6, 150}, {REST, 300}, {E6, 450}, {A6, 300}, {B6, 300}, {As6, 150}, {A6, 300}, {G6, 200},
  {E7, 200}, {G7, 200}, {A7, 300}, {F7, 150}, {G7, 150}, {REST, 150}, {E7, 300}, {C7, 150},
  {D7, 150}, {B6, 450}, {REST, 300}, {G7, 150}, {Fs7, 150}, {F7, 150}, {D7, 300}, {E7, 150},
  {REST, 150}, {G6, 150}, {A6, 150}, {C7, 150}, {REST, 150}, {A6, 150}, {C7, 150}, {D7, 150},
  {REST, 300}, {G7, 150}, {Fs7, 150}, {F7, 150}, {D7, 300}, {E7, 150}, {REST, 150}, {C5, 300},
  {C5, 150}, {C5, 300}, {REST, 300}, {REST, 300}, {G7, 150}, {Fs7, 150}, {F7, 150}, {D7, 300},
  {E7, 150}, {REST, 150}, {G6, 150}, {A6, 150}, {C7, 150}, {REST, 150}, {A6, 150}, {C7, 150},
  {D7, 150}, {REST, 300}, {Ds7, 300}, {REST, 150}, {D7, 450}, {C7, 600}, {REST, 600}, {C7, 150},
  {C7, 300}, {C7, 150}, {REST, 150}, {C7, 150}, {D7, 300}, {E7, 150}, {C7, 300}, {A6, 150},
  {G6, 600}, {C7, 150}, {C7, 300}, {C7, 150}, {REST, 150}, {C7, 150}, {D7, 150}, {E7, 150},
  {REST, 1200}, {C7, 150}, {C7, 300}, {C7, 150}, {REST, 150}, {C7, 150}, {D7, 300}, {E7, 150},
  {C7, 300}, {A6, 150}, {G6, 600}, {E7, 150}, {E7, 150}, {REST, 150}, {E7, 150}, {REST, 150},
  {C7, 150}, {E7, 300}, {G7, 300}, {REST, 300}, {G6, 300}, {REST, 300}, {C7, 450}, {G6, 150},
  {REST, 300}, {E6, 450}, {A6, 300}, {B6, 300}, {As6, 150}, {A6, 300}, {G6, 200}, {E7, 200},
  {G7, 200}, {A7, 300}, {F7, 150}, {G7, 150}, {REST, 150}, {E7, 300}, {C7, 150}, {D7, 150},
  {B6, 450}, {C7, 450}, {G6, 150}, {REST, 300}, {E6, 450}, {A6, 300}, {B6, 300}, {As6, 150},
  {A6, 300}, {G6, 200}, {E7, 200}, {G7, 200}, {A7, 300}, {F7, 150}, {G7, 150}, {REST, 150},
  {E7, 300}, {C7, 150}, {D7, 150}, {B6, 450}, {E7, 150}, {C7, 300}, {G6, 150}, {REST, 300},
  {G6, 300}, {A6, 150}, {F7, 300}, {F7, 150}, {A6, 600}, {B6, 200}, {A7, 200}, {A7, 200},
  {A7, 200}, {G7, 200}, {F7, 200}, {E7, 150}, {C7, 300}, {A6, 150}, {G6, 600}, {E7, 150},
  {C7, 300}, {G6, 150}, {REST, 300}, {G6, 300}, {A6, 150}, {F7, 300}, {F7, 150}, {A6, 600},
  {B6, 150}, {F7, 300}, {F7, 150}, {F7, 200}, {E7, 200}, {D7, 200}, {C7, 150}, {G6, 300},
  {E6, 150}, {C6, 600}, {C7, 450}, {G6, 450}, {E6, 300}, {A6, 200}, {B6, 200}, {A6, 200},
  {Gs6, 200}, {As6, 200}, {Gs6, 200}, {G6, 150}, {Fs6, 150}, {G6, 900},
  TUNE_END
};


/** Tarrega, Gran Vals, the phrase known as the Nokia ringtone (public domain) */
static const struct buzzer_note tune_gran_vals[] = {
  {E6, 160}, {D6, 160}, {Fs5, 320}, {Gs5, 320}, {Cs6, 160}, {B5, 160}, {D5, 320}, {E5, 320},
  {B5, 160}, {A5, 160}, {Cs5, 320}, {E5, 320}, {A5, 960}, TUNE_END
};

/** Beethoven, Fur Elise, first section (public domain) */
static const struct buzzer_note tune_fur_elise[] = {
  {E6, 150}, {Ds6, 150}, {E6, 150}, {Ds6, 150}, {E6, 150}, {B5, 150}, {D6, 150}, {C6, 150},
  {A5, 300}, {REST, 150}, {C5, 150}, {E5, 150}, {A5, 150}, {B5, 300}, {REST, 150},
  {E5, 150}, {Gs5, 150}, {B5, 150}, {C6, 300}, {REST, 150}, {E5, 150}, {E6, 150}, {Ds6, 150},
  {E6, 150}, {Ds6, 150}, {E6, 150}, {B5, 150}, {D6, 150}, {C6, 150},
  {A5, 300}, {REST, 150}, {C5, 150}, {E5, 150}, {A5, 150}, {B5, 300}, {REST, 150},
  {E5, 150}, {C6, 150}, {B5, 150}, {A5, 600}, TUNE_END
};

/** La Cucaracha, chorus (traditional, public domain) */
static const struct buzzer_note tune_la_cucaracha[] = {
  {C6, 150}, {C6, 150}, {C6, 150}, {F6, 450}, {A6, 300},
  {C6, 150}, {C6, 150}, {C6, 150}, {F6, 450}, {A6, 600},
  {F6, 300}, {F6, 150}, {E6, 150}, {E6, 150}, {D6, 150}, {D6, 150}, {C6, 600},
  {C6, 150}, {C6, 150}, {C6, 150}, {E6, 450}, {G6, 300},
  {C6, 150}, {C6, 150}, {C6, 150}, {E6, 450}, {G6, 600},
  {C7, 300}, {D7, 150}, {C7, 150}, {As6, 150}, {A6, 150}, {G6, 150}, {F6, 600}, TUNE_END
};

/** Rossini, William Tell Overture, galop opening, played twice (public domain) */
static const struct buzzer_note tune_william_tell[] = {
  {G6, 100}, {G6, 100}, {G6, 200}, {G6, 100}, {G6, 100}, {G6, 200},
  {G6, 100}, {G6, 100}, {C7, 200}, {D7, 200}, {E7, 400}, {REST, 200},
  {G6, 100}, {G6, 100}, {G6, 200}, {G6, 100}, {G6, 100}, {G6, 200},
  {G6, 100}, {G6, 100}, {C7, 200}, {D7, 200}, {E7, 400}, {C7, 600}, TUNE_END
};

static const struct buzzer_note *const buzzer_tunes_list[BUZZER_TUNES_CUSTOM_1] = {
  [BUZZER_TUNES_NONE]               = NULL,
  [BUZZER_TUNES_STARTUP]            = tune_startup,
  [BUZZER_TUNES_ODE_TO_JOY]         = tune_ode_to_joy,
  [BUZZER_TUNES_TWINKLE]            = tune_twinkle,
  [BUZZER_TUNES_ANTHEM_ITALY]       = tune_anthem_italy,
  [BUZZER_TUNES_ANTHEM_NETHERLANDS] = tune_anthem_netherlands,
  [BUZZER_TUNES_ANTHEM_FRANCE]      = tune_anthem_france,
  [BUZZER_TUNES_HAPPY_BIRTHDAY]     = tune_happy_birthday,
  [BUZZER_TUNES_MUSHROOM]           = tune_mushroom,
  [BUZZER_TUNES_GRAN_VALS]          = tune_gran_vals,
  [BUZZER_TUNES_FUR_ELISE]          = tune_fur_elise,
  [BUZZER_TUNES_LA_CUCARACHA]       = tune_la_cucaracha,
  [BUZZER_TUNES_WILLIAM_TELL]       = tune_william_tell,
};

/** Custom tunes, RTTTL strings */
static const char *const buzzer_tunes_custom[] = {
  BUZZER_TUNES_RTTTL_1, BUZZER_TUNES_RTTTL_2, BUZZER_TUNES_RTTTL_3
};

uint8_t buzzer_tunes_tune = BUZZER_TUNES_NONE;

static const struct buzzer_note *tune_notes = NULL; ///< next note of a built-in tune
static const char *rtttl = NULL;                    ///< next note of a RTTTL tune
static uint8_t rtttl_d = 4, rtttl_o = 6;            ///< RTTTL default duration and octave
static uint16_t rtttl_bpm = 63;                     ///< RTTTL tempo
static struct buzzer_note note;                     ///< note being played
static bool playing = false;
static uint32_t note_start_ms = 0;                  ///< start time of the note
static bool note_gap = false;                       ///< in the silent gap at the end of the note

static uint16_t rtttl_number(const char **p)
{
  uint16_t n = 0;
  while (**p >= '0' && **p <= '9') {
    n = n * 10 + (*(*p)++ - '0');
  }
  return n;
}

/** Parse the RTTTL header "name:d=4,o=6,b=120:", false if invalid */
static bool rtttl_start(const char *str)
{
  const char *p = str;
  while (*p && *p != ':') { p++; }   // skip name
  if (*p != ':') { return false; }
  p++;
  rtttl_d = 4; rtttl_o = 6; rtttl_bpm = 63;
  while (*p && *p != ':') {
    char key = *p;
    if (key == 'd' || key == 'o' || key == 'b') {
      p += 2;  // skip "x="
      uint16_t v = rtttl_number(&p);
      if (key == 'd' && v > 0) { rtttl_d = v; }
      if (key == 'o' && v > 0) { rtttl_o = v; }
      if (key == 'b' && v > 0) { rtttl_bpm = v; }
    } else {
      p++;
    }
  }
  if (*p != ':') { return false; }
  rtttl = p + 1;
  return true;
}

/** Parse the next RTTTL note "[duration]note[#][.][octave][.]", false at the end */
static bool rtttl_next(struct buzzer_note *n)
{
  // semitone frequencies of octave 4 [Hz], C to B
  static const uint16_t octave4[12] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
  static const int8_t semitone['h' - 'a' + 1] = {9, 11, 0, 2, 4, 5, 7, 11};  // a b c d e f g h(=b)
  const char *p = rtttl;
  while (*p == ' ' || *p == ',') { p++; }
  if (*p == '\0') { return false; }
  uint16_t d = rtttl_number(&p);
  if (d == 0) { d = rtttl_d; }
  char c = *p | 0x20;  // lower case
  if (c != 'p' && (c < 'a' || c > 'h')) { return false; }
  p++;
  int8_t semi = (c == 'p') ? -1 : semitone[c - 'a'];
  if (*p == '#') { semi++; p++; }
  bool dotted = false;
  if (*p == '.') { dotted = true; p++; }
  uint8_t o = rtttl_o;
  if (*p >= '0' && *p <= '9') { o = *p++ - '0'; }
  if (*p == '.') { dotted = true; p++; }
  while (*p && *p != ',') { p++; }  // ignore anything else
  rtttl = p;

  uint32_t ms = (240000UL / rtttl_bpm) / d;  // whole note = 4 beats
  if (dotted) { ms = ms * 3 / 2; }
  n->ms = Min(ms, 65535);
  if (semi < 0) {
    n->freq = REST;
  } else {
    uint8_t oct = o + semi / 12;  // B# goes to the next octave
    uint32_t f = octave4[semi % 12];
    n->freq = (oct >= 4) ? (f << (oct - 4)) : (f >> (4 - oct));
  }
  return true;
}

/** Get the next note of the tune, false at the end */
static bool next_note(struct buzzer_note *n)
{
  if (tune_notes != NULL) {
    if (tune_notes->ms == 0) { return false; }
    *n = *tune_notes++;
    return true;
  }
  if (rtttl != NULL) {
    return rtttl_next(n);
  }
  return false;
}

static void start_next_note(void)
{
  if (!next_note(&note)) {
    buzzer_tunes_play(BUZZER_TUNES_NONE);  // end of tune
    return;
  }
  note_start_ms = get_sys_time_msec();
  note_gap = false;
  buzzer_arch_tone(note.freq);
}

void buzzer_tunes_play(uint8_t tune)
{
  playing = false;
  tune_notes = NULL;
  rtttl = NULL;
  buzzer_arch_tone(0);
  buzzer_tunes_tune = BUZZER_TUNES_NONE;
  if (tune == BUZZER_TUNES_NONE || tune >= BUZZER_TUNES_NB) {
    return;
  }
  if (tune >= BUZZER_TUNES_CUSTOM_1) {
    if (!rtttl_start(buzzer_tunes_custom[tune - BUZZER_TUNES_CUSTOM_1])) {
      return;  // empty or invalid custom tune
    }
  } else {
    tune_notes = buzzer_tunes_list[tune];
  }
  buzzer_tunes_tune = tune;
  playing = true;
  start_next_note();
}

void buzzer_tunes_init(void)
{
  buzzer_arch_init();
  buzzer_tunes_play(BUZZER_TUNES_STARTUP_TUNE);
}

void buzzer_tunes_periodic(void)
{
  if (!playing) {
    return;
  }
  uint32_t elapsed = get_sys_time_msec() - note_start_ms;
  if (!note_gap && elapsed + BUZZER_TUNES_GAP_MS >= note.ms) {
    buzzer_arch_tone(0);
    note_gap = true;
  }
  if (elapsed >= note.ms) {
    start_next_note();
  }
}
