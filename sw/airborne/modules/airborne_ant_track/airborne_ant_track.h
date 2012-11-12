/*
 * Copyright (C) 2005-2008  Arnold Schroeter
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

#if defined(USE_AIRBORNE_ANT_TRACKING) && USE_AIRBORNE_ANT_TRACKING == 1
#ifndef AIRBORNE_ANT_TRACK_H
#define AIRBORNE_ANT_TRACK_H

extern float   airborne_ant_pan;
void airborne_ant_point_init(void);
void airborne_ant_point_periodic(void);

#endif /* AIRBORNE_ANT_TRACK_H */
#endif // #if defined(USE_AIRBORNE_ANT_TRACKING) && USE_AIRBORNE_ANT_TRACKING == 1
