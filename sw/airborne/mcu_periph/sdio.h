/*
 * Copyright (C) 2015 Gautier Hattenberger, Alexandre Bustico
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/** \file mcu_periph/sdio.h
 *  \brief arch independent SDIO API
 *
 */

#ifndef MCU_PERIPH_SDIO_H
#define MCU_PERIPH_SDIO_H

/** Connect a SD card on SDIO peripheral
 */
bool sdio_connect(void);

/** Disconnect a SD card on SDIO peripheral
 */
bool sdio_disconnect(void);

/** Check if a SD card is inserted
 */
bool is_card_inserted  (void);

#endif

