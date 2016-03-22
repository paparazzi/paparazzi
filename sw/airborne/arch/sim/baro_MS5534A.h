/*
 * Copyright (C) 2007  ENAC
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

/**
 * @file arch/sim/baro_MS5534A.h
 *
 * Dummy Handling of the MS5534a pressure sensor for the sim.
 *
 */

#ifndef BARO_MS5534A_H
#define BARO_MS5534A_H

#include "std.h"

#if USE_BARO_MS5534A

extern bool spi_message_received;
extern bool baro_MS5534A_available;
extern uint32_t baro_MS5534A_pressure;
extern uint16_t baro_MS5534A_temp;
extern bool alt_baro_enabled;
extern uint32_t baro_MS5534A_ground_pressure;
extern float baro_MS5534A_r;
extern float baro_MS5534A_sigma2;
extern float baro_MS5534A_z;


void baro_MS5534A_init(void);
void baro_MS5534A_reset(void);

/* To be called not faster than 30Hz */
void baro_MS5534A_send(void);

/* Set baro_MS5534A_available when pressure and temp are readable */
void baro_MS5534A_event_task(void);

#endif // USE_BARO_MS5534A

#endif // BARO_MS5534A_H
