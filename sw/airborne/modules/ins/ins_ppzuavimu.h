/*
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

#ifndef PPZUAVIMU_H
#define PPZUAVIMU_H

#include "std.h"
// pitch/roll neutrals
#include "ins_module.h"

extern int32_t mag_x, mag_y, mag_z;
extern int32_t gyr_x, gyr_y, gyr_z;
extern int32_t acc_x, acc_y, acc_z;

extern void ppzuavimu_module_init( void );
extern void ppzuavimu_module_periodic( void );
extern void ppzuavimu_module_event( void );

#endif // PPZUAVIMU_H
