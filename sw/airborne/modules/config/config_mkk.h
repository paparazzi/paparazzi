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


struct config_mkk_struct
{
    int addr;
    int temp;
    int current;

    int nb_err;

    uint8_t read_nr;
    struct i2c_transaction trans;
};

extern struct config_mkk_struct config_mkk;

#define config_mkk_SetCommand(_v) {   \
    config_mkk.addr = _v;             \
}





void init_config_mkk(void);
void periodic_config_mkk_read_status(void);
void periodic_config_mkk_telemetry(void);


#endif



