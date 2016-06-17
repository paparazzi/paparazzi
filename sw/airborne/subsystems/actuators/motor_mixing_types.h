/*
 * Copyright (C) 2008-2015 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file motor_mixing_types.h
 *  Common Motor Mixing configuration types.
 */

#ifndef MOTOR_MIXING_TYPES_H
#define MOTOR_MIXING_TYPES_H

/* already defined common configurations*/
#define QUAD_PLUS   1
#define QUAD_X      2
#define HEXA_X      3
#define HEXA_PLUS   4
#define OCTO_X      5
#define OCTO_PLUS   6

/* set to TRUE to reverse the rotation direction of all motors */
#if MOTOR_MIXING_REVERSE
#define MOTOR_YAW_SIGN (-1)
#else
#define MOTOR_YAW_SIGN 1
#endif

#if MOTOR_MIXING_TYPE == QUAD_PLUS
/*
 * Quadrotor in plus (+) cross configuration with motor order:
 * front (CW), right (CCW), back (CW), left (CCW)
 */
#define MOTOR_FRONT 0
#define MOTOR_RIGHT 1
#define MOTOR_BACK  2
#define MOTOR_LEFT  3
#define MOTOR_MIXING_NB_MOTOR    4
#define MOTOR_MIXING_SCALE       256
#define MOTOR_MIXING_ROLL_COEF   {    0, -256,    0,  256 }
#define MOTOR_MIXING_PITCH_COEF  {  256,    0, -256,    0 }
#define MOTOR_MIXING_YAW_COEF    { -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128, -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128 }
#define MOTOR_MIXING_THRUST_COEF {  256,  256,  256,  256 }

#elif MOTOR_MIXING_TYPE == QUAD_X
/*
 * Quadrotor in time cross (X) configuration with motor order:
 * front left (CW), front right (CCW), back right (CW), back left (CCW)
 */
#define MOTOR_FRONT_LEFT  0
#define MOTOR_FRONT_RIGHT 1
#define MOTOR_BACK_RIGHT  2
#define MOTOR_BACK_LEFT   3
#define MOTOR_MIXING_NB_MOTOR    4
#define MOTOR_MIXING_SCALE       256
#define MOTOR_MIXING_ROLL_COEF   {  181, -181, -181,  181 }
#define MOTOR_MIXING_PITCH_COEF  {  181,  181, -181, -181 }
#define MOTOR_MIXING_YAW_COEF    { -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128, -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128 }
#define MOTOR_MIXING_THRUST_COEF {  256,  256,  256,  256 }

#elif  MOTOR_MIXING_TYPE == HEXA_X
/*
 * Hexarotor in time cross (X) configuration with motor order:
 * front left (CW), front right (CCW), right (CW), back right (CCW), back left (CW), left (CCW)
 */
#define MOTOR_FRONT_LEFT  0
#define MOTOR_FRONT_RIGHT 1
#define MOTOR_RIGHT       2
#define MOTOR_BACK_RIGHT  3
#define MOTOR_BACK_LEFT   4
#define MOTOR_LEFT        5
#define MOTOR_MIXING_NB_MOTOR    6
#define MOTOR_MIXING_SCALE       256
#define MOTOR_MIXING_ROLL_COEF   {  128, -128, -256, -128,  128,  256 }
#define MOTOR_MIXING_PITCH_COEF  {  222,  222,    0, -222, -222,    0 }
#define MOTOR_MIXING_YAW_COEF    { -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128, -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128, -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128 }
#define MOTOR_MIXING_THRUST_COEF {  256,  256,  256,  256,  256,  256 }

#elif  MOTOR_MIXING_TYPE == HEXA_PLUS
/*
 * Hexarotor in plus (+) configuration with motor order:
 * front (CW), front right (CCW), back right (CW), back (CCW), back left (CW), front left (CCW)
 */
#define MOTOR_FRONT       0
#define MOTOR_FRONT_RIGHT 1
#define MOTOR_BACK_RIGHT  2
#define MOTOR_BACK        3
#define MOTOR_BACK_LEFT   4
#define MOTOR_FRONT_LEFT  5
#define MOTOR_MIXING_NB_MOTOR    6
#define MOTOR_MIXING_SCALE       256
#define MOTOR_MIXING_ROLL_COEF   {    0, -222, -222,    0,  222,  222 }
#define MOTOR_MIXING_PITCH_COEF  {  256,  128, -128, -256, -128,  128 }
#define MOTOR_MIXING_YAW_COEF    { -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128, -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128, -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128 }
#define MOTOR_MIXING_THRUST_COEF {  256,  256,  256,  256,  256,  256 }

#elif  MOTOR_MIXING_TYPE == OCTO_PLUS
/*
 * Hexarotor in plus cross (+) configuration with motor order:
 * front (CW), front right (CCW), right (CW), back right (CCW),
 * back (CW), back left (CCW), left (CW), front left (CCW)
 */
#define MOTOR_FRONT       0
#define MOTOR_FRONT_RIGHT 1
#define MOTOR_RIGHT       2
#define MOTOR_BACK_RIGHT  3
#define MOTOR_BACK        4
#define MOTOR_BACK_LEFT   5
#define MOTOR_LEFT        6
#define MOTOR_FRONT_LEFT  7
#define MOTOR_MIXING_NB_MOTOR    8
#define MOTOR_MIXING_SCALE       256
#define MOTOR_MIXING_ROLL_COEF   {    0, -181, -256, -181,    0,  181,  256,  181 }
#define MOTOR_MIXING_PITCH_COEF  {  256,  181,    0, -181, -256, -181,    0,  181 }
#define MOTOR_MIXING_YAW_COEF    { -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128, -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128, -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128, -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128 }
#define MOTOR_MIXING_THRUST_COEF {  256,  256,  256,  256,  256,  256,  256,  256 }

#elif  MOTOR_MIXING_TYPE == OCTO_X
/*
 * Hexarotor in time cross (X) configuration with motor order:
 * front left1 (CW), front right1 (CCW), front right2 (CW), back right1 (CCW),
 * back right2 (CW), back left1 (CCW), back left2 (CW), front left2 (CCW)
 */
#define MOTOR_FRONT_LEFT1  0
#define MOTOR_FRONT_RIGHT1 1
#define MOTOR_FRONT_RIGHT2 2
#define MOTOR_BACK_RIGHT1  3
#define MOTOR_BACK_RIGHT2  4
#define MOTOR_BACK_LEFT1   5
#define MOTOR_BACK_LEFT2   6
#define MOTOR_FRONT_LEFT2  7
#define MOTOR_MIXING_NB_MOTOR    8
#define MOTOR_MIXING_SCALE       256
#define MOTOR_MIXING_ROLL_COEF   {   98,  -98, -237, -237,  -98,   98,  237,  237 }
#define MOTOR_MIXING_PITCH_COEF  {  237,  237,   98,  -98, -237, -237,  -98,   98 }
#define MOTOR_MIXING_YAW_COEF    { -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128, -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128, -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128, -MOTOR_YAW_SIGN*128,  MOTOR_YAW_SIGN*128 }
#define MOTOR_MIXING_THRUST_COEF {  256,  256,  256,  256,  256,  256,  256,  256 }

#endif

#endif /* MOTOR_MIXING_TYPES_H */
