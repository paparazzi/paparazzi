/*
 * Copyright (C) 2024 Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef ACTUATORS_T4_H
#define ACTUATORS_T4_H

#include "modules/t4/t4.h"
#include BOARD_CONFIG

/* External functions */
extern void actuators_t4_init(struct t4_iface_t *iface);
extern void actuators_t4_commit(struct t4_iface_t *iface, int16_t *values, uint8_t nb);
extern void actuators_t4_cmd_commit(struct t4_iface_t *iface, int16_t *values, uint8_t nb);

#endif /* ACTUATORS_T4_H */
