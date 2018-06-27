/*
 * Copyright (C) 2017 Ewoud Smeur <ewoud_smeur@msn.com>
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

/** @file modules/ctrl/ctrl_effectiveness_scheduling.c
 * Module that interpolates gainsets in flight based on the transition percentage
 */

#include "modules/ctrl/ctrl_effectiveness_scheduling.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/radio_control.h"

#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
#error "You need to use WLS control allocation for this module"
#endif


static float g1g2_forward[INDI_OUTPUTS][INDI_NUM_ACT] = {FWD_G1_ROLL,
                                                  FWD_G1_PITCH, FWD_G1_YAW, FWD_G1_THRUST
                                                 };

static float g1g2_hover[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
                                                STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST
                                               };

static float g2_both[INDI_NUM_ACT] = STABILIZATION_INDI_G2; //scaled by INDI_G_SCALING

//Get the specified gains in the gainlibrary
void ctrl_eff_scheduling_init(void)
{
  //sum of G1 and G2
  int8_t i;
  int8_t j;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    for (j = 0; j < INDI_NUM_ACT; j++) {
      if (i != 2) {
        g1g2_hover[i][j] = g1g2_hover[i][j] / INDI_G_SCALING;
        g1g2_forward[i][j] = g1g2_forward[i][j] / INDI_G_SCALING;
      } else {
        g1g2_forward[i][j] = (g1g2_forward[i][j] + g2_both[j]) / INDI_G_SCALING;
        g1g2_hover[i][j] = (g1g2_hover[i][j] + g2_both[j]) / INDI_G_SCALING;
      }
    }
  }
}

void ctrl_eff_scheduling_periodic(void)
{
  // Go from transition percentage to ratio
  float ratio = FLOAT_OF_BFP(transition_percentage, INT32_PERCENTAGE_FRAC) / 100;

  int8_t i;
  int8_t j;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    for (j = 0; j < INDI_NUM_ACT; j++) {
      g1g2[i][j] = (g1g2_hover[i][j] * (1.0 - ratio) + g1g2_forward[i][j] * ratio);
    }
  }
}
