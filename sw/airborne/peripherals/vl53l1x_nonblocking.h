/*
 * Copyright (C) 2020 Tom van Dijk <tomvand@users.noreply.github.com>
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
/** @file peripherals/vl53l1x_nonblocking.h
 *  @brief Non-blocking runtime functions for the VL53L1X.
 */

#ifndef VL53L1X_NONBLOCKING_H
#define VL53L1X_NONBLOCKING_H


#include "vl53l1x_api.h"

#include <stdbool.h>


/**
 * @brief This function checks if the new ranging data is available by polling the dedicated register.
 * @param : isDataReady==0 -> not ready; isDataReady==1 -> ready
 * @return: TRUE upon completion
 */
bool VL53L1X_NonBlocking_CheckForDataReady(VL53L1_DEV dev, uint8_t *isDataReady);

/**
 * @brief This function returns the distance measured by the sensor in mm
 * @return: TRUE upon completion
 */
bool VL53L1X_NonBlocking_GetDistance(VL53L1_DEV dev, uint16_t *distance);

/**
 * @brief This function clears the interrupt, to be called after a ranging data reading
 * to arm the interrupt for the next data ready event.
 * @return: TRUE upon completion
 */
bool VL53L1X_NonBlocking_ClearInterrupt(VL53L1_DEV dev);


#endif // VL53L1X_NONBLOCKING_H
