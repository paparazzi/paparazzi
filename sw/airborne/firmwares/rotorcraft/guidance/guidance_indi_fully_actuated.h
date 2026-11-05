/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *               2025 Mohamad Hachem
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi_fully_actuated.h
 *
 * Fully actuated plateform can be achieve with hexa-copter with
 * tilted propellers for example
 * TODO cite Mohamad Hachem
 */

#ifndef GUIDANCE_INDI_FULLY_ACTUATED
#define GUIDANCE_INDI_FULLY_ACTUATED

#include "firmwares/rotorcraft/stabilization.h"

/** Set horizontal thrust from RC
 *  Horizontal is passed as an argument
 */
extern struct ThrustSetpoint guidance_set_rc_h_thrust(struct ThrustSetpoint *v_sp);

/** Set flat attitude setpoint from RC
 *  Heading setpoint is kept
 */
extern struct StabilizationSetpoint guidance_set_rc_h_att(struct StabilizationSetpoint *a_sp);

#endif

