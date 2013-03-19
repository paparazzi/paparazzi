/*
    vbat.c - AR.Drone battery voltage driver

    Copyright (C) 2011 Hugo Perquin - http://blog.perquin.com

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/
#ifndef _VBAT_H
#define _VBAT_H

typedef struct {
	float vbat;
	float vdd0;
	float vdd1;
	float vdd2;
	float vdd3;
	float vdd4;
	float vdd0_setpoint;
	float vdd1_setpoint;
	float vdd2_setpoint;
	float vdd3_setpoint;
	float vdd4_setpoint;
} vbat_struct;

float vbat_get(unsigned char channel);
int vbat_init(vbat_struct *vbat);
void vbat_read(vbat_struct *vbat);

#endif
