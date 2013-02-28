/*
 * Copyright (C) 2013 Christophe De Wagter
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

/** \file config_mkk.h
 *
 * Read Status and Config from MKK (Mikrokopter) BLDC motor controllers
 */

#ifndef CONFIG_MKK_MODULE_H
#define CONFIG_MKK_MODULE_H

#include "std.h"

#include "mcu_periph/i2c.h"

typedef struct
{
  uint8_t revision;
  uint8_t SetMask;
  uint8_t PwmScaling;
  uint8_t CurrentLimit;
  uint8_t TempLimit;
  uint8_t CurrentScaling;
  uint8_t BitConfig;
  uint8_t crc;
} config_mkk_eeprom_t;

extern config_mkk_eeprom_t config_mkk_eeprom;

struct config_mkk_struct
{
    int read_config;
    int addr;

    int nb_err;

    uint8_t read_nr;
    struct i2c_transaction trans;
};

extern struct config_mkk_struct config_mkk;

extern void config_mkk_send_eeprom(void);

#define config_mkk_SetConfig(_v) {    \
    config_mkk.addr = _v;             \
    config_mkk_send_eeprom();         \
}

#define config_mkk_GetConfig(_v) {    \
    config_mkk.addr = _v;             \
    config_mkk.read_config = 1;       \
}



void init_config_mkk(void);
void periodic_config_mkk_read_status(void);
void periodic_config_mkk_telemetry(void);


#endif



