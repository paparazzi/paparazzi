/*
 * Copyright (C) 2017  Michal Podhradsky
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

/** \file modules/fsae_electric/vms_ecu_demo.h
 *
 * Viking Motorsports Engine Control Unit demo module
 * see https://wiki.paparazziuav.org/wiki/VMS_ECU
 * for more details
 */
#ifndef DRIVERS_TEST_H
#define DRIVERS_TEST_H

#include "std.h"
#include "mcu_periph/adc.h"

#if !USE_CHIBIOS_RTOS
#error Only Chibios is supported
#endif

// Definitions of pins

extern bool ams_status;
extern bool pwr_ready;
extern bool pwr_stdby;
extern bool rtds;

void vms_ecu_demo_init(void);
void vms_ecu_demo_periodic(void);
/** Reset sweep number */
extern void vms_ecu_demo_UpdateDac1(uint16_t val);
extern void vms_ecu_demo_UpdateDac2(uint16_t val);
extern uint16_t dac_1;
extern uint16_t dac_2;

#endif /* DRIVERS_TEST */
