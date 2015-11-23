/*
 * Copyright (C) 2015 C. De Wagter
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/helicopter/swashplate_mixing.c"
 * @author C. De Wagter and Freek van Tienen
 * Helicopter Swashplate Mixing
 */

#include "swashplate_mixing.h"
#include "throttle_curve.h"

PRINT_CONFIG_VAR(SW_MIXING_TYPE)
struct swashplate_mixing_t swashplate_mixing;

/* Coeficients per motor */
static const float roll_coef[SW_NB]   = SW_MIXING_ROLL_COEF;
static const float pitch_coef[SW_NB]  = SW_MIXING_PITCH_COEF;
static const float coll_coef[SW_NB]   = SW_MIXING_COLL_COEF;

/* Default roll trim */
#ifndef SW_MIXING_TRIM_ROLL
#define SW_MIXING_TRIM_ROLL 0
#endif
PRINT_CONFIG_VAR(SW_MIXING_TRIM_ROLL)

/* Default pitch trim */
#ifndef SW_MIXING_TRIM_PITCH
#define SW_MIXING_TRIM_PITCH 0
#endif
PRINT_CONFIG_VAR(SW_MIXING_TRIM_PITCH)

/* Default collective trim */
#ifndef SW_MIXING_TRIM_COLL
#define SW_MIXING_TRIM_COLL 0
#endif
PRINT_CONFIG_VAR(SW_MIXING_TRIM_COLL)


/**
 * Initialize the motor mixing and calculate the trim values
 */
void swashplate_mixing_init()
{
  uint8_t i;

  // Go trough all the motors and calculate the trim value and set the initial command
  for (i = 0; i < SW_NB; i++) {
    swashplate_mixing.commands[i] = 0;
    swashplate_mixing.trim[i] =
      roll_coef[i]  * SW_MIXING_TRIM_ROLL +
      pitch_coef[i] * SW_MIXING_TRIM_PITCH +
      coll_coef[i]  * SW_MIXING_TRIM_COLL;
  }
}

/*
 * Run the swashplate mixing
 * This depends on the ROLL and PITCH command
 * It also depends on the throttle_curve.collective
 */
void swashplate_mixing_run(pprz_t in_cmd[])
{
  uint8_t i;

  // Go trough all the motors and calculate the command
  for (i = 0; i < SW_NB; i++) {
    swashplate_mixing.commands[i] = swashplate_mixing.trim[i] +
        roll_coef[i] * in_cmd[COMMAND_ROLL] +
        pitch_coef[i] * in_cmd[COMMAND_PITCH] +
        coll_coef[i] * throttle_curve.collective;
    BoundAbs(swashplate_mixing.commands[i], MAX_PPRZ);
  }
}
