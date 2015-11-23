/*
 * Copyright (C) 2015 The Paparazzi Team
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
 *
 */

/** @file subsystems/intermcu/intermcu_fbw.h
 *  @brief Rotorcraft Inter-MCU on FlyByWire
 */

#ifndef INTERMCU_FBW_ROTORCRAFT_H
#define INTERMCU_FBW_ROTORCRAFT_H

#include "subsystems/intermcu.h"

extern pprz_t intermcu_commands[COMMANDS_NB];
void intermcu_on_rc_frame(void);
void intermcu_send_status(uint8_t mode);
void InterMcuEvent(void (*frame_handler)(void));

#endif /* INTERMCU_FBW_ROTORCRAFT_H */
