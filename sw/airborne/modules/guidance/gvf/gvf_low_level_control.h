/*
 * Copyright (C) 2021 Hector Garcia de Marina <hgarciad@ucm.es>
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
 * @file modules/guidance/gvf/gvf_low_level_control.h
 *
 * Firmware dependent file for the classic guiding vector field algorithm
 */

#ifndef GVF_LOW_LEVEL_CONTROL_H
#define GVF_LOW_LEVEL_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

// Low level control functions
extern void gvf_low_level_control(float);

#ifdef __cplusplus
}
#endif


#endif // GVF_LOW_LEVEL_CONTROL_H

