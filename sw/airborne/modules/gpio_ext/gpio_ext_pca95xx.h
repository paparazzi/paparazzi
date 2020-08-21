/*
 * Copyright (C) Tom van Dijk <tomvand@users.noreply.github.com>
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

/** @file "modules/gpio_ext/gpio_ext_pca95xx.h"
 * @author Tom van Dijk <tomvand@users.noreply.github.com>
 * PCA95XX external GPIO peripheral
 */

#ifndef GPIO_EXT_PCA95XX_H
#define GPIO_EXT_PCA95XX_H

#define GPIO_EXT_PCA95XX &pca95xx_functions

extern struct gpio_ext_functions pca95xx_functions;

#endif  // GPIO_EXT_PCA95XX_H
