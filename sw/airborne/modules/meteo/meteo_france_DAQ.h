/*
 * Copyright (C) 2014 Gautier Hattenberger
 *
 * This file is part of paparazzi

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
 * @file modules/meteo/meteo_france_DAQ.h
 *
 * Communication module with the Data Acquisition board
 * from Meteo France
 *
 * DAQ board sends measurments to the AP
 * AP sends periodic report to the ground, store data on SD card
 * and sends A/C state to DAQ board
 */

#ifndef METEO_FRANCE_DAQ_H
#define METEO_FRANCE_DAQ_H

#include "std.h"
#include "mcu_periph/gpio.h"
#include "generated/airframe.h"

#define MF_DAQ_SIZE 32

struct MF_DAQ {
  float values[MF_DAQ_SIZE];
  uint8_t nb;
  uint8_t power;
};

extern struct MF_DAQ mf_daq;

extern void init_mf_daq(void);
extern void mf_daq_send_state(void);
extern void mf_daq_send_report(void);
extern void parse_mf_daq_msg(void);

#if (defined MF_DAQ_POWER_PORT) && (defined MF_DAQ_POWER_PIN)
#define meteo_france_DAQ_SetPower(_x) { \
    mf_daq.power = _x; \
    if (mf_daq.power) { gpio_set(MF_DAQ_POWER_PORT, MF_DAQ_POWER_PIN); } \
    else { gpio_clear(MF_DAQ_POWER_PORT, MF_DAQ_POWER_PIN); } \
  }
#else // POWER PORT and PIN undefined
#define meteo_france_DAQ_SetPower(_x) {}
#endif

#endif

