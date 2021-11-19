/*
 * Copyright (C) 2016 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
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
 * @file ahrs_vectornav_wrapper.h
 *
 * Vectornav VN-200 as AHRS
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef AHRS_VECTORNAV_WRAPPER_H
#define AHRS_VECTORNAV_WRAPPER_H

#include "modules/ahrs/ahrs_vectornav.h"

#ifndef PRIMARY_AHRS
#define PRIMARY_AHRS ahrs_vectornav
#endif

extern void ahrs_vectornav_register(void);
extern bool ahrs_vectornav_is_enabled(void);

#endif /* AHRS_VECTORNAV_WRAPPER_H */
