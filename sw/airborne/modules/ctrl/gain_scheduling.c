/*
 * Copyright (C) 2012 Pranay Sinha <psinha@transition-robotics.com>
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

/** @file modules/ctrl/gain_scheduling.c
 * Module that interpolates gainsets in flight based on a scheduling variable
 */

#include "modules/ctrl/gain_scheduling.h"

//Include for scheduling on transition_status
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/stabilization.h"

// #include "state.h"
#include "math/pprz_algebra_int.h"

#ifndef NUMBER_OF_GAINSETS
#error You must define the number of gainsets to use this module!
#endif

#ifndef SCHEDULING_VARIABLE_FRAC
#define SCHEDULING_VARIABLE_FRAC 0
#pragma message "SCHEDULING_VARIABLE_FRAC not specified!"
#endif

#define INT32_RATIO_FRAC 12

struct Int32AttitudeGains gainlibrary[NUMBER_OF_GAINSETS];

float scheduling_points[NUMBER_OF_GAINSETS] = SCHEDULING_POINTS;

//Get the specified gains in the gainlibrary
void gain_scheduling_init(void)
{
  int32_t phi_p[NUMBER_OF_GAINSETS] = PHI_P;
  int32_t phi_d[NUMBER_OF_GAINSETS] = PHI_D;
  int32_t phi_i[NUMBER_OF_GAINSETS] = PHI_I;
  int32_t phi_dd[NUMBER_OF_GAINSETS] = PHI_DD;

  int32_t theta_p[NUMBER_OF_GAINSETS] = THETA_P;
  int32_t theta_d[NUMBER_OF_GAINSETS] = THETA_D;
  int32_t theta_i[NUMBER_OF_GAINSETS] = THETA_I;
  int32_t theta_dd[NUMBER_OF_GAINSETS] = THETA_DD;

  int32_t psi_p[NUMBER_OF_GAINSETS] = PSI_P;
  int32_t psi_d[NUMBER_OF_GAINSETS] = PSI_D;
  int32_t psi_i[NUMBER_OF_GAINSETS] = PSI_I;
  int32_t psi_dd[NUMBER_OF_GAINSETS] = PSI_DD;

  for (int i = 0; i < NUMBER_OF_GAINSETS; i++) {

    struct Int32AttitudeGains swap = {
      {phi_p[i], theta_p[i], psi_p[i] },
      {phi_d[i], theta_d[i], psi_d[i] },
      {phi_dd[i], theta_dd[i], psi_dd[i] },
      {phi_i[i], theta_i[i], psi_i[i] }
    };

    gainlibrary[i] = swap;
  }
  stabilization_gains = gainlibrary[0];
}

void gain_scheduling_periodic(void)
{

#if NUMBER_OF_GAINSETS > 1
  uint8_t section = 0;

  //Find out between which gainsets to interpolate
  while (FLOAT_OF_BFP(SCHEDULING_VARIABLE, SCHEDULING_VARIABLE_FRAC) > scheduling_points[section]) {
    section++;
    if (section == NUMBER_OF_GAINSETS) { break; }
  }

  //Get pointers for the two gainsets and the stabilization_gains
  struct Int32AttitudeGains *ga, *gb, *gblend;

  gblend = &stabilization_gains;

  if (section == 0) {
    set_gainset(0);
  } else if (section == NUMBER_OF_GAINSETS) {
    set_gainset(NUMBER_OF_GAINSETS - 1);
  } else {
    ga = &gainlibrary[section - 1];
    gb = &gainlibrary[section];

    //Calculate the ratio between the scheduling points
    int32_t ratio;
    ratio = BFP_OF_REAL((FLOAT_OF_BFP(SCHEDULING_VARIABLE,
                                      SCHEDULING_VARIABLE_FRAC) - scheduling_points[section - 1]) / (scheduling_points[section] -
                                          scheduling_points[section - 1]), INT32_RATIO_FRAC);

    int64_t g1, g2, gbl;

    //Loop through the gains and interpolate
    for (int i = 0; i < (sizeof(struct Int32AttitudeGains) / sizeof(int32_t)); i++) {
      g1 = *(((int32_t *) ga) + i);
      g1 *= (1 << INT32_RATIO_FRAC) - ratio;
      g2 = *(((int32_t *) gb) + i);
      g2 *= ratio;

      gbl = (g1 + g2) >> INT32_RATIO_FRAC;

      *(((int32_t *) gblend) + i) = (int32_t) gbl;
    }
  }
#endif
}

//Set one of the gainsets entirely
void set_gainset(int gainset)
{
  stabilization_gains = gainlibrary[gainset];
}
