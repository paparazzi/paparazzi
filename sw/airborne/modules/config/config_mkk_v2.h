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

/** \file config_mkk_v2.h
 *
 * Read Status and Config from MKK (Mikrokopter) BLDC motor controllers
 */

#ifndef config_mkk_v2_MODULE_H
#define config_mkk_v2_MODULE_H

#include "std.h"

#include "mcu_periph/i2c.h"

struct config_mkk_v2_struct {
  uint8_t read_config;
  uint8_t addr;

  int nb_err;

  struct i2c_transaction trans;
};

extern struct config_mkk_v2_struct config_mkk_v2;

void config_mkk_v2_init(void);
void config_mkk_v2_periodic_read_status(void);
void config_mkk_v2_periodic_telemetry(void);

//////////////////////////////////////////////////////////////////
// MKK Config

typedef struct {
  uint8_t revision;
  uint8_t SetMask;
  uint8_t PwmScaling;
  uint8_t CurrentLimit;
  uint8_t TempLimit;
  uint8_t CurrentScaling;
  uint8_t BitConfig;
  uint8_t crc;
} config_mkk_v2_eeprom_t;

extern config_mkk_v2_eeprom_t config_mkk_v2_eeprom;


#define CONFIG_MKK_V2_MASK_SET_PWM_SCALING       0x01
#define CONFIG_MKK_V2_MASK_SET_CURRENT_LIMIT     0x02
#define CONFIG_MKK_V2_MASK_SET_TEMP_LIMIT        0x04
#define CONFIG_MKK_V2_MASK_SET_CURRENT_SCALING   0x08
#define CONFIG_MKK_V2_MASK_SET_BITCONFIG         0x10
#define CONFIG_MKK_V2_MASK_RESET_CAPCOUNTER      0x20
#define CONFIG_MKK_V2_MASK_SET_DEFAULT_PARAMS    0x40
#define CONFIG_MKK_V2_MASK_SET_SAVE_EEPROM       0x80

#define BITCONF_REVERSE_ROTATION   0x01


extern void config_mkk_v2_send_eeprom(void);
extern void config_mkk_v2_read_eeprom(void);

#define config_mkk_v2_ResetDefault(_v) {      \
    config_mkk_v2_eeprom.SetMask = CONFIG_MKK_V2_MASK_SET_SAVE_EEPROM | CONFIG_MKK_V2_MASK_SET_DEFAULT_PARAMS; \
    config_mkk_v2_send_eeprom();         \
  }

#define config_mkk_v2_SetPwmScaling(_v) {      \
    config_mkk_v2_eeprom.PwmScaling = _v;             \
    config_mkk_v2_eeprom.SetMask = CONFIG_MKK_V2_MASK_SET_SAVE_EEPROM | CONFIG_MKK_V2_MASK_SET_PWM_SCALING; \
    config_mkk_v2_send_eeprom();         \
  }

#define config_mkk_v2_SetCurrentLimit(_v) {      \
    config_mkk_v2_eeprom.CurrentLimit = _v;             \
    config_mkk_v2_eeprom.SetMask = CONFIG_MKK_V2_MASK_SET_SAVE_EEPROM | CONFIG_MKK_V2_MASK_SET_CURRENT_LIMIT; \
    config_mkk_v2_send_eeprom();         \
  }

#define config_mkk_v2_SetTempLimit(_v) {      \
    config_mkk_v2_eeprom.TempLimit = _v;             \
    config_mkk_v2_eeprom.SetMask = CONFIG_MKK_V2_MASK_SET_SAVE_EEPROM | CONFIG_MKK_V2_MASK_SET_TEMP_LIMIT; \
    config_mkk_v2_send_eeprom();         \
  }

#define config_mkk_v2_SetCurrentScaling(_v) {      \
    config_mkk_v2_eeprom.CurrentScaling = _v;             \
    config_mkk_v2_eeprom.SetMask = CONFIG_MKK_V2_MASK_SET_SAVE_EEPROM | CONFIG_MKK_V2_MASK_SET_CURRENT_SCALING; \
    config_mkk_v2_send_eeprom();         \
  }

#define config_mkk_v2_SetBitConfig(_v) {      \
    config_mkk_v2_eeprom.BitConfig = _v;             \
    config_mkk_v2_eeprom.SetMask = CONFIG_MKK_V2_MASK_SET_SAVE_EEPROM | CONFIG_MKK_V2_MASK_SET_BITCONFIG; \
    config_mkk_v2_send_eeprom();         \
  }

#define config_mkk_v2_GetConfig(_v) {      \
    config_mkk_v2.addr = _v;               \
    config_mkk_v2_read_eeprom();             \
  }



#endif



