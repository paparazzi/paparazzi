/*
 * Copyright (C) 2015 Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file arch/chibios/mcu_periph/gpio_def.h
 *
 * ChibiOS doesn't define pin numbers with format GPIOX
 * let's do it here
 */
#ifndef GPIO_DEF_H
#define GPIO_DEF_H

#define GPIO0    0
#define GPIO1    1
#define GPIO2    2
#define GPIO3    3
#define GPIO4    4
#define GPIO5    5
#define GPIO6    6
#define GPIO7    7
#define GPIO8    8
#define GPIO9    9
#define GPIO10   10
#define GPIO11   11
#define GPIO12   12
#define GPIO13   13
#define GPIO14   14
#define GPIO15   15
#define GPIO16   16
#define GPIO17   17
#define GPIO18   18
#define GPIO19   19
#define GPIO20   20
#define GPIO21   21
#define GPIO22   22
#define GPIO23   23
#define GPIO24   24
#define GPIO25   25
#define GPIO26   26
#define GPIO27   27
#define GPIO28   28
#define GPIO29   29
#define GPIO30   30
#define GPIO31   31

#define GPIO_AF0 0
#define GPIO_AF1 1
#define GPIO_AF2 2
#define GPIO_AF3 3
#define GPIO_AF4 4
#define GPIO_AF5 5
#define GPIO_AF6 6
#define GPIO_AF7 7
#define GPIO_AF8 8
#define GPIO_AF9 9
#define GPIO_AF10 10
#define GPIO_AF11 11
#define GPIO_AF12 12
#define GPIO_AF13 13
#define GPIO_AF14 14
#define GPIO_AF15 15

#endif /* GPIO_DEF_H */

