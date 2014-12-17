/*
 * Copyright (C) 2007-2009  ENAC, Pascal Brisset, Antoine Drouin
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
 * @file subsystems/navigation/common_flight_plan.c
 * Common flight_plan functions shared between fixedwing and rotorcraft.
 */

#include "subsystems/navigation/common_flight_plan.h"

#include "generated/flight_plan.h"


/** In s */
uint16_t stage_time, block_time;

uint8_t nav_stage, nav_block;

/** To save the current block/stage to enable return */
uint8_t last_block, last_stage;


void nav_init_block(void)
{
  if (nav_block >= NB_BLOCK) {
    nav_block = NB_BLOCK - 1;
  }
  nav_stage = 0;
  block_time = 0;
  InitStage();
}

void nav_goto_block(uint8_t b)
{
  if (b != nav_block) { /* To avoid a loop in a the current block */
    last_block = nav_block;
    last_stage = nav_stage;
  }
  GotoBlock(b);
}
