/*
 * Copyright (C) 2010 Martin Mueller
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

#ifndef TRIGGER_EXT_HW_H
#define TRIGGER_EXT_HW_H

#include "core/trigger_ext.h"

// Default trigger Pin is PPM pin (Tiny2/Twog)
// To use a custom trigger, you must set the flag USE_CUSTOM_TRIGGER
// and define:
// - PINSEL
// - PINSEL_VAL
// - PINSEL_BIT
// - input capture CHANNEL
#ifndef USE_CUSTOM_TRIGGER
#define TRIG_EXT_PINSEL     PPM_PINSEL
#define TRIG_EXT_PINSEL_VAL PPM_PINSEL_VAL
#define TRIG_EXT_PINSEL_BIT PPM_PINSEL_BIT
#define TRIG_EXT_CHANNEL    2
#endif

#define __SelectCapReg(_c) T0CR ## _c
#define _SelectCapReg(_c) __SelectCapReg(_c)
#define SelectCapReg(_c) _SelectCapReg(_c)

#define __SetIntFlag(_c) TIR_CR ## _c ## I
#define _SetIntFlag(_c) __SetIntFlag(_c)
#define SetIntFlag(_c) _SetIntFlag(_c)

#define __EnableRise(_c) TCCR_CR ## _c ## _R
#define _EnableRise(_c) __EnableRise(_c)
#define EnableRise(_c) _EnableRise(_c)

#define __EnableFall(_c) TCCR_CR ## _c ## _F
#define _EnableFall(_c) __EnableFall(_c)
#define EnableFall(_c) _EnableFall(_c)

#define __EnableInt(_c) TCCR_CR ## _c ## _I
#define _EnableInt(_c) __EnableInt(_c)
#define EnableInt(_c) _EnableInt(_c)

#define TRIGGER_CR SelectCapReg(TRIG_EXT_CHANNEL)
#define TRIGGER_IT SetIntFlag(TRIG_EXT_CHANNEL)
#define TRIGGER_CRR EnableRise(TRIG_EXT_CHANNEL)
#define TRIGGER_CRF EnableFall(TRIG_EXT_CHANNEL)
#define TRIGGER_CRI EnableInt(TRIG_EXT_CHANNEL)

/* Interrupt function called by sys_time_hw.c */
void TRIG_ISR(void);

#endif /* TRIGGER_EXT_HW_H */

