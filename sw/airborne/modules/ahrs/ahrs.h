/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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
 * @file modules/ahrs/ahrs.h
 */

#ifndef AHRS_H
#define AHRS_H

#include "std.h"

#define AHRS_DISABLE    0
#define AHRS_PRIMARY    1
#define AHRS_SECONDARY  2

#define AHRS_COMP_ID_NONE       0
#define AHRS_COMP_ID_GENERIC    1
#define AHRS_COMP_ID_IR         2
#define AHRS_COMP_ID_ICQ        3
#define AHRS_COMP_ID_ICE        4
#define AHRS_COMP_ID_FC         4
#define AHRS_COMP_ID_DCM        6
#define AHRS_COMP_ID_FINV       7
#define AHRS_COMP_ID_MLKF       8
#define AHRS_COMP_ID_GX3        9
// CHIMU (10) is not supported anymore
#define AHRS_COMP_ID_VECTORNAV 11
#define AHRS_COMP_ID_EKF2      12
#define AHRS_COMP_ID_MADGWICK  13
#define AHRS_COMP_ID_FLOW      14

/** AHRS initialization.
 */
extern void ahrs_init(void);

#endif /* AHRS_H */
