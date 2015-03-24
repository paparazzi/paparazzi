/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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

/**
 * @file subsystems/abi_sender_ids.h
 *
 * Convenience defines for ABI sender IDs.
 */

#ifndef ABI_SENDER_IDS_H
#define ABI_SENDER_IDS_H

/** default onboard baro */
#ifndef BARO_BOARD_SENDER_ID
#define BARO_BOARD_SENDER_ID 1
#endif

/*
 * IDs of baro modules that can be loaded
 */
#ifndef BARO_MS5611_SENDER_ID
#define BARO_MS5611_SENDER_ID 10
#endif

#ifndef BARO_AMSYS_SENDER_ID
#define BARO_AMSYS_SENDER_ID 11
#endif

#ifndef BARO_BMP_SENDER_ID
#define BARO_BMP_SENDER_ID 12
#endif

#ifndef BARO_ETS_SENDER_ID
#define BARO_ETS_SENDER_ID 13
#endif

#ifndef BARO_MS5534A_SENDER_ID
#define BARO_MS5534A_SENDER_ID 14
#endif

#ifndef BARO_HCA_SENDER_ID
#define BARO_HCA_SENDER_ID 15
#endif

#ifndef BARO_MPL3115_SENDER_ID
#define BARO_MPL3115_SENDER_ID 16
#endif

#ifndef BARO_SCP_SENDER_ID
#define BARO_SCP_SENDER_ID 17
#endif

#ifndef BARO_PBN_SENDER_ID
#define BARO_PBN_SENDER_ID 18
#endif

#ifndef BARO_SIM_SENDER_ID
#define BARO_SIM_SENDER_ID 19
#endif

#ifndef METEO_STICK_SENDER_ID
#define METEO_STICK_SENDER_ID 30
#endif

/*
 * IDs of differential pressure sensors
 * can usually also publish temperature like baro sensors
 */
#ifndef MS45XX_SENDER_ID
#define MS45XX_SENDER_ID 40
#endif

/*
 * IDs of AGL measurment modules that can be loaded (sonars,...)
 */
#ifndef AGL_SONAR_ADC_ID
#define AGL_SONAR_ADC_ID 1
#endif

#ifndef AGL_SONAR_ARDRONE2_ID
#define AGL_SONAR_ARDRONE2_ID 2
#endif

#ifndef AGL_SONAR_NPS_ID
#define AGL_SONAR_NPS_ID 3
#endif

/*
 * IDs of magnetometer sensors (including IMUs with mag)
 */

#ifndef MAG_HMC58XX_SENDER_ID
#define MAG_HMC58XX_SENDER_ID 2
#endif

#endif /* ABI_SENDER_IDS_H */
