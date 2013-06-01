/*
 * Copyright (C) 2011 Hugo Perquin - http://blog.perquin.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA.
 */

/**
 * @file boards/ardrone/gpio.h
 * ardrone GPIO driver
 */

#ifndef GPIO_H
#define GPIO_H

//val=0 -> set gpio output lo
//val=1 -> set gpio output hi
//val=-1 -> set gpio as input (output hi-Z)
int gpio_set(int nr,int val);

#endif /* GPIO_H */
