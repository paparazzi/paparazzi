/*
 * $Id: nav_cube.h 1936 2007-10-23 12:12:38Z hecto $
 *
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
 *
 */

#ifndef NAV_CUBE_H
#define NAV_CUBE_H

#include "std.h"

#define MAX_LINES_X 8
#define STBY_OFFSET 500

extern bool_t nav_cube_init(uint8_t turb, uint8_t tb, uint8_t te);
bool_t nav_cube(int8_t j, int8_t i,
                uint8_t dest_b, uint8_t dest_e,
                uint8_t src_b, uint8_t src_e);

extern int32_t cube_alpha;     /* angle of flight direction to north, clockwise */
extern int32_t cube_size_x;    /* size of the cube x (perpendicular to flight dir) */
extern int32_t cube_size_y;    /* size of the cube y (in flight dir) */
extern int32_t cube_size_z;    /* height of the cube z */
extern int32_t cube_grid_x;    /* grid distance x (horizontal) */
extern int32_t cube_grid_z;    /* grid distance z (vertical) */
extern int32_t cube_offs_x;    /* offset to center x (horizontal) */
extern int32_t cube_offs_y;    /* offset to center y (in direction) */
extern int32_t cube_offs_z;    /* offset to center z (vertical) */

extern int32_t cube_sect;      /* sector to fly in (1..[nsect_x*nsect_z]) */
extern int32_t cube_nsect_x;   /* number of sectors horizontal */
extern int32_t cube_nsect_z;   /* number of sectors vertical */

extern int32_t cube_nline_x;   /* number of lines x (horizontal) */
extern int32_t cube_nline_z;   /* number of lines z (vertical) */

#define nav_cube_SetAlpha(i) { cube_alpha=i; }
#define nav_cube_SetSect(i)  { cube_sect=i; }
#define nav_cube_SetGridX(i) { cube_grid_x=i; }
#define nav_cube_SetGridZ(i) { cube_grid_z=i; }
#define nav_cube_SetSizeX(i) { cube_size_x=i; }
#define nav_cube_SetSizeY(i) { cube_size_y=i; }
#define nav_cube_SetSizeZ(i) { cube_size_z=i; }
#define nav_cube_SetOffsX(i) { cube_offs_x=i; }
#define nav_cube_SetOffsY(i) { cube_offs_y=i; }
#define nav_cube_SetOffsZ(i) { cube_offs_z=i; }
#define nav_cube_SetNSectX(i) { cube_nsect_x=i; }
#define nav_cube_SetNSectZ(i) { cube_nsect_z=i; }

/*

 from above:

         | (offs_x)

         o [x, y]    -
                     |
                     | (offs_y)
                     |
                     -
   ^  ^  ^  ^  ^     |
   |  |  |  |  |     |
   |  |  |  |  |     | (size_y)
   |  |  |  |  |     |
   |  |  |  |  |     |
                     -
   |--|
    (grid_x)

   |-----------|
     (size_x)


 side view:

   --------->                    -           -
                                 | (grid_z)  |
   --------->                    -           |
                                 -           | (size_z)
   --------->                    | (offs_z)  |
                      o [alt]    -           |
   --------->                                -

  |----------|--------|
    (size_y)  (offs_y)


 back view, 1x1 sector:

   -------
  |       |
  |   1   |  ^
  |       |  |
  s-------   |

  ---->


 back view, 3x2 sectors:

   ------- ------- -------
  |       |       |       |
  |   4   |   5   |   6   |  ^
  |       |       |       |  |
  s-------s-------s-------   |
  |       |       |       |
  |   1   |   2   |   3   |  ^
  |       |       |       |  |
  s-------s-------s-------   | (nsect_z)

  ---->   ---->   ---->
   (nsect_x)

*/

#endif /* NAV_CUBE_H */
