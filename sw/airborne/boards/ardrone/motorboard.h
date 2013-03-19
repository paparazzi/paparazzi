/*
    motorboard.h - AR.Drone motor driver

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
#ifndef _MOTORBOARD_H
#define _MOTORBOARD_H

#include <stdint.h>

#define MOT_LEDOFF 0
#define MOT_LEDRED 1
#define MOT_LEDGREEN 2
#define MOT_LEDORANGE 3

int motorboard_cmd(uint8_t cmd, uint8_t *reply, int replylen);
int motorboard_Init(void);
void motorboard_SetPWM(uint16_t pwm0, uint16_t pwm1, uint16_t pwm2, uint16_t pwm3);
void motorboard_SetLeds(uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3);
void motorboard_Close(void);

#endif
