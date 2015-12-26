/*
 * Copyright (C) 2013  Christophe De Wagter
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

#include "config_mkk_v2.h"
#include "generated/airframe.h"
#include "subsystems/actuators.h"

void config_mkk_v2_parse_eeprom(void);

struct config_mkk_v2_struct config_mkk_v2;

void config_mkk_v2_init(void)
{
  config_mkk_v2.nb_err = 0;
  config_mkk_v2.read_config = 0;

  config_mkk_v2.trans.status = I2CTransSuccess;

}

#include "subsystems/actuators/actuators_mkk_v2.h"

void config_mkk_v2_periodic_read_status(void)
{
  // Read Config
  if (config_mkk_v2.read_config > 0) {
    switch (config_mkk_v2.trans.status) {
      case I2CTransFailed:
        config_mkk_v2.read_config = 0;
        break;
      case I2CTransSuccess:
      case I2CTransDone:
        config_mkk_v2_parse_eeprom();
        config_mkk_v2.read_config = 0;
        break;
      default:
        return;
    }
  }
}

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"


void config_mkk_v2_periodic_telemetry(void)
{
  static uint8_t send_nr = 0;

  DOWNLINK_SEND_MKK(DefaultChannel, DefaultDevice, &send_nr, &actuators_mkk_v2.data[send_nr].MaxPWM,
                    &actuators_mkk_v2.data[send_nr].Current, &actuators_mkk_v2.data[send_nr].Temperature);

  send_nr++;
  if (send_nr >= ACTUATORS_MKK_V2_NB) {
    send_nr = 0;
  }
}



//////////////////////////////////////////////////////////////////
// MKK Config

uint8_t config_mkk_v2_crc(uint8_t offset);

// Following 2 structs are known from:  http://mikrokopter.de/mikrosvn/FlightCtrl/tags/V0.88n/twimaster.h

/*
typedef struct
{
  uint8_t revision;     // must be BL_revision
  uint8_t SetMask;      // settings mask
  uint8_t PwmScaling;     // maximum value of control pwm, acts like a thrust limit
  uint8_t CurrentLimit;   // current limit in A
  uint8_t TempLimit;      // in °C
  uint8_t CurrentScaling;   // scaling factor for current measurement
  uint8_t BitConfig;      // see defines above
  uint8_t crc;        // checksum
}  __attribute__((packed)) config_mkk_v2_eeprom_t;

extern config_mkk_v2_eeprom_t config_mkk_v2_eeprom;
*/

uint8_t config_mkk_v2_crc(uint8_t offset)
{
  uint8_t crc = 0xaa;
  for (int i = offset; i < (offset + 7); i++) {
    crc += config_mkk_v2.trans.buf[i];
  }
  return crc;
}


config_mkk_v2_eeprom_t config_mkk_v2_eeprom;


#define BL_READMODE_CONFIG              16
#define config_mkk_v2_EEPROM_REVISION      2


#define RETURN_IF_NOT_KILLMODE() \
  {                                \
    if (!actuators_delay_done)     \
      return;                      \
  }

void config_mkk_v2_read_eeprom(void)
{
  // Activate decoder
  config_mkk_v2.read_config = 1;

  // Do not read config while running
  RETURN_IF_NOT_KILLMODE();

  // New I2C Write/Read Transaction
  config_mkk_v2.trans.type = I2CTransTxRx;
  config_mkk_v2.trans.slave_addr = 0x52 + config_mkk_v2.addr * 2;
  config_mkk_v2.trans.len_w = 2;
  config_mkk_v2.trans.buf[0] = 0;
  config_mkk_v2.trans.buf[1] = (BL_READMODE_CONFIG << 3);
  config_mkk_v2.trans.len_r = 8;

  i2c_submit(&ACTUATORS_MKK_V2_I2C_DEV, &config_mkk_v2.trans);
}

void config_mkk_v2_parse_eeprom(void)
{
  config_mkk_v2_eeprom.crc            = config_mkk_v2.trans.buf[7];   // checksum
  if (config_mkk_v2_crc(0) != config_mkk_v2_eeprom.crc) {
    config_mkk_v2.nb_err++;
  } else {
    config_mkk_v2_eeprom.revision       = config_mkk_v2.trans.buf[0];   // must be BL_revision
    config_mkk_v2_eeprom.SetMask        = config_mkk_v2.trans.buf[1];   // settings mask
    config_mkk_v2_eeprom.PwmScaling     =
      config_mkk_v2.trans.buf[2];   // maximum value of control pwm, acts like a thrust limit
    config_mkk_v2_eeprom.CurrentLimit   = config_mkk_v2.trans.buf[3];   // current limit in A
    config_mkk_v2_eeprom.TempLimit      = config_mkk_v2.trans.buf[4];   // in °C
    config_mkk_v2_eeprom.CurrentScaling = config_mkk_v2.trans.buf[5];   // scaling factor for current measurement
    config_mkk_v2_eeprom.BitConfig      = config_mkk_v2.trans.buf[6];   // see defines above
  }
}

void config_mkk_v2_send_eeprom(void)
{
  // Do not upload while running
  RETURN_IF_NOT_KILLMODE();

  // Do not upload bad data:
  if (config_mkk_v2_eeprom.revision != config_mkk_v2_EEPROM_REVISION) {
    return;
  }

  // New I2C Write Transaction
  config_mkk_v2.trans.type = I2CTransTx;
  config_mkk_v2.trans.slave_addr = 0x52 + config_mkk_v2.addr * 2;
  config_mkk_v2.trans.len_w = 10;
  config_mkk_v2.trans.buf[0] = 0;
  config_mkk_v2.trans.buf[1] = (BL_READMODE_CONFIG << 3);

  config_mkk_v2.trans.buf[2] = config_mkk_v2_eeprom.revision;
  config_mkk_v2.trans.buf[3] = config_mkk_v2_eeprom.SetMask;
  config_mkk_v2.trans.buf[4] = config_mkk_v2_eeprom.PwmScaling;
  config_mkk_v2.trans.buf[5] = config_mkk_v2_eeprom.CurrentLimit;
  config_mkk_v2.trans.buf[6] = config_mkk_v2_eeprom.TempLimit;
  config_mkk_v2.trans.buf[7] = config_mkk_v2_eeprom.CurrentScaling;
  config_mkk_v2.trans.buf[8] = config_mkk_v2_eeprom.BitConfig;
  config_mkk_v2.trans.buf[9] = config_mkk_v2_crc(2);

  i2c_submit(&ACTUATORS_MKK_V2_I2C_DEV, &config_mkk_v2.trans);

}
