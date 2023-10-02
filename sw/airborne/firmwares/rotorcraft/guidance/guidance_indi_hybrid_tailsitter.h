/*
 * Copyright (C) 2023 TUDelft
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



// Guidance actuators: (roll, pitch, thrust) for tailsitters 
#define GUIDANCE_INDI_HYBRID_U  3

// Guidance control objectives: ax, ay, ay for tailsitters
#define GUIDANCE_INDI_HYBRID_V  3

#ifndef GUIDANCE_INDI_MIN_PITCH
#define GUIDANCE_INDI_MIN_PITCH -120
#define GUIDANCE_INDI_MAX_PITCH 25
#endif
