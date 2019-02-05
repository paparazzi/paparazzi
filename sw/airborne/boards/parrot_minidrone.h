/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

#ifndef CONFIG_PARROT_MINIDRONE
#define CONFIG_PARROT_MINIDRONE

#define BOARD_PARROT_MINIDRONE

#include "peripherals/video_device.h"

/** FTDI cable for external GPS or other periferals */
#define UART2_DEV /dev/ttyUSB0

/* Default actuators driver */
#define DEFAULT_ACTUATORS "boards/parrot_minidrone/actuators.h"
#define ActuatorDefaultSet(_x,_y) ActuatorsParrotMinidroneSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsParrotMinidroneInit()
#define ActuatorsDefaultCommit() ActuatorsParrotMinidroneCommit()

/* Cameras */
extern struct video_config_t bottom_camera;

/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

/* The ADC from the sonar */
#if USE_ADC0
#define ADC0_ID             0
#define ADC0_CHANNELS       2
#define ADC0_CHANNELS_CNT   1
#define ADC0_BUF_LENGTH     8192
#endif

/* The SPI from the sonar */
#if USE_SPI0
#define SPI0_MODE           0
#define SPI0_BITS_PER_WORD  8
#define SPI0_MAX_SPEED_HZ   320000
#endif

#endif /* CONFIG_PARROT_MINIDRONE */

