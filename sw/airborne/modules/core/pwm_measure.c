/*
 * $Id$
 *
 * Copyright (C) 2011 Stephen Dwyer, based on trigger_ext by Martin Mueller
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

/** \file pwm_measure.c
 *  \brief Measure external pwm pulse at a capture input
 *
 *   This measures the duty active
 */


#include "core/pwm_measure.h"

volatile uint32_t *pwm_meas_duty_tics; // this is an alias for the hw dep array, array length linked to PWM_INPUT_NB
volatile uint8_t *pwm_meas_duty_valid; // this is an alias for the hw dep array, array length linked to PWM_INPUT_NB
