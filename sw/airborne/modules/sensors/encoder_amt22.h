/*
 * Copyright (C) 2025 Flo&Fab <name.surname@enac.fr>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/sensors/encoder_amt22.h"
 * @author Flo&Fab <name.surname@enac.fr>
 * Driver for AMT22 encoder from CUI devices.
 */

#ifndef ENCODER_AMT22_H
#define ENCODER_AMT22_H

extern void encoder_amt22_init(void);
extern void encoder_amt22_periodic(void);

#endif  // ENCODER_AMT22_H
