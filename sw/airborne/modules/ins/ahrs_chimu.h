/*
 * Copyright (C) 2014 Felix Ruess
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
 * @file modules/ins/ahrs_chimu.h
 */

#ifndef AHRS_CHIMU_H
#define AHRS_CHIMU_H

#include "modules/ins/ins_module.h"
#include "subsystems/ahrs.h"

struct AhrsChimu {
  bool_t is_aligned;
};

extern struct AhrsChimu ahrs_chimu;

#ifndef PRIMARY_AHRS
#define PRIMARY_AHRS ahrs_chimu
#endif

extern void ahrs_chimu_register(void);
extern void ahrs_chimu_init(void);

#endif
