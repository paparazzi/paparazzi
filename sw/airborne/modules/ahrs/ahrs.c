/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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
 * @file modules/ahrs/ahrs.c
 * Dispatcher to register actual AHRS implementations.
 */


#include "modules/ahrs/ahrs.h"

#if USE_AHRS_ALIGNER
#include "modules/ahrs/ahrs_aligner.h"
#endif


#ifndef PRIMARY_AHRS
#error "PRIMARY_AHRS not set!"
#else
PRINT_CONFIG_VAR(PRIMARY_AHRS)
#endif

#ifdef SECONDARY_AHRS
PRINT_CONFIG_VAR(SECONDARY_AHRS)
#endif

#define __RegisterAhrs(_x) _x ## _register()
#define _RegisterAhrs(_x) __RegisterAhrs(_x)
#define RegisterAhrs(_x) _RegisterAhrs(_x)

/** maximum number of AHRS implementations that can register */
#ifndef AHRS_NB_IMPL
#define AHRS_NB_IMPL 2
#endif

/** references a registered AHRS implementation */
struct AhrsImpl {
  AhrsEnableOutput enable;
};

struct AhrsImpl ahrs_impls[AHRS_NB_IMPL];
uint8_t ahrs_output_idx;

void ahrs_register_impl(AhrsEnableOutput enable)
{
  int i;
  for (i=0; i < AHRS_NB_IMPL; i++) {
    if (ahrs_impls[i].enable == NULL) {
      ahrs_impls[i].enable = enable;
      break;
    }
  }
}

void ahrs_init(void)
{
  int i;
  for (i=0; i < AHRS_NB_IMPL; i++) {
    ahrs_impls[i].enable = NULL;
  }

  RegisterAhrs(PRIMARY_AHRS);
#ifdef SECONDARY_AHRS
  RegisterAhrs(SECONDARY_AHRS);
#endif

  // enable primary AHRS by default
  ahrs_switch(0);

#if USE_AHRS_ALIGNER
  ahrs_aligner_init();
#endif
}

int ahrs_switch(uint8_t idx)
{
  if (idx >= AHRS_NB_IMPL) { return -1; }
  if (ahrs_impls[idx].enable == NULL) { return -1; }
  /* first disable other AHRS output */
  int i;
  for (i=0; i < AHRS_NB_IMPL; i++) {
    if (ahrs_impls[i].enable != NULL) {
      ahrs_impls[i].enable(FALSE);
    }
  }
  /* enable requested AHRS */
  ahrs_impls[idx].enable(TRUE);
  ahrs_output_idx = idx;
  return ahrs_output_idx;
}
