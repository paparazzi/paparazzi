/* $Id: ppm.h 4281 2009-10-19 18:32:12Z mmm $
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

#ifndef TRIG_EXT_H
#define TRIG_EXT_H

#if defined TRIGGER_EXT

#include "std.h"
extern uint32_t trigger_t0;
extern uint32_t delta_t0;
extern volatile bool_t trig_ext_valid;

#include "trig_ext_hw.h"

#endif /* TRIGGER_EXT */

#endif
