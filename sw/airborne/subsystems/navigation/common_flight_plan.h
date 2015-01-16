/*
 * Copyright (C) 2007-2011  The Paparazzi Team
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
 * @file subsystems/navigation/common_flight_plan.h
 * Common flight_plan functions shared between fixedwing and rotorcraft.
 */

#ifndef COMMON_FLIGHT_PLAN_H
#define COMMON_FLIGHT_PLAN_H

#include "std.h"

/** In s */
extern uint16_t stage_time, block_time;

extern uint8_t nav_stage, nav_block;
extern uint8_t last_block, last_stage;

/** needs to be implemented by fixedwing and rotorcraft seperately */
void nav_init_stage(void);

void nav_init_block(void);
void nav_goto_block(uint8_t block_id);

#define InitStage() nav_init_stage();

#define Block(x) case x: nav_block=x;
#define NextBlock() { nav_block++; nav_init_block(); }
#define GotoBlock(b) { nav_block=b; nav_init_block(); }

#define Stage(s) case s: nav_stage=s;
#define NextStage() { nav_stage++; InitStage(); }
#define NextStageAndBreak() { nav_stage++; InitStage(); break; }
#define NextStageAndBreakFrom(wp) { last_wp = wp; NextStageAndBreak(); }

#define Label(x) label_ ## x:
#define Goto(x) { goto label_ ## x; }
#define Return() { nav_block=last_block; nav_stage=last_stage; block_time=0;}

#define And(x, y) ((x) && (y))
#define Or(x, y) ((x) || (y))
#define Min(x,y) (x < y ? x : y)
#define Max(x,y) (x > y ? x : y)
#define LessThan(_x, _y) ((_x) < (_y))
#define MoreThan(_x, _y) ((_x) > (_y))

/** Time in s since the entrance in the current block */
#define NavBlockTime() (block_time)

#endif /* COMMON_FLIGHT_PLAN_H */
