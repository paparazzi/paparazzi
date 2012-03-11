/*
 * This file is part of mathlib.
 *
 * Copyright (C) 2010-2011 Greg Horn <ghorn@stanford.edu>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*
 * misc_math.c
 */

#include "misc_math.h"


// this function is deprecated and will soon be removed
// please use the BOUND macro instead, it can be cound in misc_math.h
double bound(const double in, const double min, const double max)
{
	float ret = in;
	if(ret<min) ret = min;
	if(ret>max) ret = max;
	return ret;
}

