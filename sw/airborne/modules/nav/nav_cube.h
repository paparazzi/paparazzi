/*
 * Copyright (C) 2010  Martin Mueller
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
 * @file modules/nav/nav_cube.h
 *
 * Fixedwing Navigation in a cube towards a center.
 *
 * @verbatim
 * from above:
 *
 * | (offs_x)
 *
 * o [x, y]    -
 * |
 * | (offs_y)
 * |
 * -
 * ^  ^  ^  ^  ^     |
 * |  |  |  |  |     |
 * |  |  |  |  |     | (size_y)
 * |  |  |  |  |     |
 * |  |  |  |  |     |
 * -
 * |--|
 * (grid_x)
 *
 * |-----------|
 * (size_x)
 *
 *
 * side view:
 *
 * --------->                    -           -
 * | (grid_z)  |
 * --------->                    -           |
 * -           | (size_z)
 * --------->                    | (offs_z)  |
 * o [alt]    -           |
 * --------->                                -
 *
 * |----------|--------|
 * (size_y)  (offs_y)
 *
 *
 * back view, 1x1 sector:
 *
 * -------
 * |       |
 * |   1   |  ^
 * |       |  |
 * s-------   |
 *
 * ---->
 *
 *
 * back view, 3x2 sectors:
 *
 * ------- ------- -------
 * |       |       |       |
 * |   4   |   5   |   6   |  ^
 * |       |       |       |  |
 * s-------s-------s-------   |
 * |       |       |       |
 * |   1   |   2   |   3   |  ^
 * |       |       |       |  |
 * s-------s-------s-------   | (nsect_z)
 *
 * ---->   ---->   ---->
 * (nsect_x)
 * @endverbatim
 *
 */

#ifndef NAV_CUBE_H
#define NAV_CUBE_H

#include "std.h"
#include "math/pprz_algebra_int.h"

struct NavCube {
  /** size of the cube.
   * x: perpendicular to flight dir,
   * y: in flight dir,
   * z: height
   */
  struct Int32Vect3 size;

  /** offset to center.
   * x: horizontal,
   * y: in direction,
   * z: vertical
   */
  struct Int32Vect3 offset;
  int32_t alpha;            ///< angle in degrees of flight direction to north, clockwise
  int32_t grid_x;           ///< grid distance x (horizontal)
  int32_t grid_z;           ///< grid distance z (vertical)
  int32_t sect;             ///< sector to fly in (1..[nsect_x*nsect_z])
  int32_t nsect_x;          ///< number of sectors horizontal
  int32_t nsect_z;          ///< number of sectors vertical
  int32_t nline_x;          ///< number of lines x (horizontal)
  int32_t nline_z;          ///< number of lines z (vertical)
};

extern struct NavCube nav_cube;

extern bool nav_cube_setup(uint8_t turb, uint8_t tb, uint8_t te);
bool nav_cube_run(int8_t j, int8_t i,
                    uint8_t dest_b, uint8_t dest_e,
                    uint8_t src_b, uint8_t src_e);

#define nav_cube_SetAlpha(i) { nav_cube.alpha=i; }
#define nav_cube_SetSect(i)  { nav_cube.sect=i; }
#define nav_cube_SetGridX(i) { nav_cube.grid_x=i; }
#define nav_cube_SetGridZ(i) { nav_cube.grid_z=i; }
#define nav_cube_SetSizeX(i) { nav_cube.size.x=i; }
#define nav_cube_SetSizeY(i) { nav_cube.size.y=i; }
#define nav_cube_SetSizeZ(i) { nav_cube.size.z=i; }
#define nav_cube_SetOffsX(i) { nav_cube.offs.x=i; }
#define nav_cube_SetOffsY(i) { nav_cube.offs.y=i; }
#define nav_cube_SetOffsZ(i) { nav_cube.offs.z=i; }
#define nav_cube_SetNSectX(i) { nav_cube.nsect_x=i; }
#define nav_cube_SetNSectZ(i) { nav_cube.nsect_z=i; }


#endif /* NAV_CUBE_H */
