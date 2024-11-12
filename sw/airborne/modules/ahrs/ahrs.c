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


/** maximum number of AHRS implementations that can register */
#ifndef AHRS_NB_IMPL
#define AHRS_NB_IMPL 2
#endif

/** references a registered AHRS implementation */
struct AhrsImpl {
  uint16_t module_id;
};

struct AhrsImpl ahrs_impls[AHRS_NB_IMPL];
uint8_t ahrs_output_idx;

void ahrs_register(uint8_t index, uint16_t module_id)
{
  if (index > AHRS_NB_IMPL) { return; }
  ahrs_impls[index].module_id = module_id;
}

void ahrs_init(void)
{
  int i;
  for (i=0; i < AHRS_NB_IMPL; i++) {
    ahrs_impls[i].module_id = 0;
  }

#if USE_AHRS_ALIGNER
  ahrs_aligner_init();
#endif
}

