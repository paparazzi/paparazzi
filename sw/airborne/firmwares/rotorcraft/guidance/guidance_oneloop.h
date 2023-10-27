/*
 * Copyright (C) 2015 Tomaso De Ponti <t.m.l.deponti@tudelft.nl>
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
 * @file firmwares/rotorcraft/guidance/guidance_oneloop.h
 *
 * A dummy guidance mode to run the oneloop_andi controller
 */

#ifndef GUIDANCE_ONELOOP_H
#define GUIDANCE_ONELOOP_H

#include "firmwares/rotorcraft/oneloop/oneloop_andi.h"

enum GuidanceOneloop_HMode {
  GUIDANCE_ONELOOP_H_POS,
  GUIDANCE_ONELOOP_H_SPEED,
  GUIDANCE_ONELOOP_H_ACCEL
};

enum GuidanceOneloop_VMode {
  GUIDANCE_ONELOOP_V_POS,
  GUIDANCE_ONELOOP_V_SPEED,
  GUIDANCE_ONELOOP_V_ACCEL
};

extern struct StabilizationSetpoint guidance_oneloop_run_mode(bool in_flight, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceOneloop_HMode h_mode, enum GuidanceOneloop_VMode v_mode);

#endif /* GUIDANCE_INDI_H */