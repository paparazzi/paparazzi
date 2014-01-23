/*
 * Copyright (C) 2013 Gautier Hattenberger, Alexandre Bustico
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
 */

#include <ch.h>
#include <hal.h>
#include "chibios_stub.h"




void chibios_chSysLock (void)
{
  chSysLock ();
}

void chibios_chSysUnlock (void)
{
  chSysUnlock ();
}

void chibios_chSysLockFromIsr (void)
{
    chSysLockFromIsr();
}

void chibios_chSysUnlockFromIsr (void)
{
    chSysUnlockFromIsr();
}

void chibios_chSysDisable (void)
{
    chSysDisable();
}

void chibios_chSysEnable (void)
{
    chSysEnable();
}

void chibios_chRegSetThreadName (const char* thdName)
{
  chRegSetThreadName (thdName);
}

void chibios_chThdSleepMilliseconds(uint32_t ms)
{
  chThdSleepMilliseconds(ms);
}

void chibios_chThdSleepMicroseconds(uint32_t us)
{
   chThdSleepMicroseconds(us);
}

uint32_t chibios_chTimeNow(void)
{
  return chTimeNow();
}
