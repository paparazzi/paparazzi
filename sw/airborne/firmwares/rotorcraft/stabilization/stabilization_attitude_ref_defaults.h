/*
 * Copyright (C) 2008-2015 The Paparazzi Team
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

/** @file firmwares/rotorcraft/stabilization/stabilization_attitude_ref_defaults.h
 *  Default values for attitude reference saturation.
 */

#ifndef ATTITUDE_REF_DEFAULTS_H
#define ATTITUDE_REF_DEFAULTS_H

#ifndef STABILIZATION_ATTITUDE_REF_MAX_P
#define STABILIZATION_ATTITUDE_REF_MAX_P RadOfDeg(400.)
#endif
#ifndef STABILIZATION_ATTITUDE_REF_MAX_Q
#define STABILIZATION_ATTITUDE_REF_MAX_Q RadOfDeg(400.)
#endif
#ifndef STABILIZATION_ATTITUDE_REF_MAX_R
#define STABILIZATION_ATTITUDE_REF_MAX_R RadOfDeg(180.)
#endif

#ifndef STABILIZATION_ATTITUDE_REF_MAX_PDOT
#define STABILIZATION_ATTITUDE_REF_MAX_PDOT RadOfDeg(2000.)
#endif
#ifndef STABILIZATION_ATTITUDE_REF_MAX_QDOT
#define STABILIZATION_ATTITUDE_REF_MAX_QDOT RadOfDeg(2000.)
#endif
#ifndef STABILIZATION_ATTITUDE_REF_MAX_RDOT
#define STABILIZATION_ATTITUDE_REF_MAX_RDOT RadOfDeg(1800.)
#endif

#endif /* ATTITUDE_REF_DEFAULTS_H */
