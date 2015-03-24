/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** \file mcu_periph/link_device.h
 *  generic device header
 */

#ifndef LINK_DEVICE_H
#define LINK_DEVICE_H

#include <inttypes.h>

/** Function pointers definition
 *
 * they are used to cast the real functions with the correct type
 * to store in the device structure
 */
typedef int (*check_free_space_t)(void *, uint8_t);
typedef void (*put_byte_t)(void *, uint8_t);
typedef void (*send_message_t)(void *);
typedef int (*char_available_t)(void *);
typedef uint8_t (*get_byte_t)(void *);

/** Device structure
 */
struct link_device {
  check_free_space_t check_free_space;  ///< check if transmit buffer is not full
  put_byte_t put_byte;                  ///< put one byte
  send_message_t send_message;          ///< send completed buffer
  char_available_t char_available;      ///< check if a new character is available
  get_byte_t get_byte;                  ///< get a new char
  void *periph;                         ///< pointer to parent implementation
};

#endif // LINK_DEVICE_H

