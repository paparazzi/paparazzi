/* $Id$
 *
 * (c) 2005 Pascal Brisset, Antoine Drouin
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

#ifndef PPM_H
#define PPM_H

#include "std.h"

#if defined RADIO_CONTROL

#include "radio.h"
#define PPM_NB_PULSES RADIO_CTL_NB
extern uint16_t ppm_pulses[ PPM_NB_PULSES ];
extern volatile bool_t	ppm_valid;

#include "ppm_hw.h"

#endif /* RADIO_CONTROL */

#endif
