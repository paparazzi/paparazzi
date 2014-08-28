/*
 * Copyright (C) 2013-2014 Piotr Esden-Tempski <piotr@esden.net>
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

#ifndef CONFIG_LISA_MX_2_1_H
#define CONFIG_LISA_MX_2_1_H

#include "boards/lisa_mx_common.h"

/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
/* (esden) Note to self. Revision 4 of the board is on GPIO0 not GPIO2.
 * Note to everyone else. Revision 4 was never produced or provided to the
 * public. :)
 */
#define SPEKTRUM_BIND_PIN GPIO2
#define SPEKTRUM_BIND_PIN_PORT GPIOB

#endif /* CONFIG_LISA_MX_2_1_H */
