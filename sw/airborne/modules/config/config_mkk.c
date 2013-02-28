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

#include "config_mkk.h"
#include "generated/airframe.h"



void config_mkk_read_eeprom(void);
void config_mkk_parse_eeprom(void);
uint8_t config_mkk_crc(uint8_t offset);


// Following 2 structs are known from:  http://mikrokopter.de/mikrosvn/FlightCtrl/tags/V0.88n/twimaster.h


struct config_mkk_struct config_mkk;

#define MAX_MOTORS ACTUATORS_MKK2_NB


typedef struct
{
  uint8_t Version;
  uint8_t Current;        // in 0.1 A steps, read back from BL
  uint8_t MaxPWM;         // read back from BL -> is less than 255 if BL is in current limit, not running (250) or starting (40)
  int8_t  Temperature;    // old BL-Ctrl will return a 255 here, the new version the temp. in °C
} __attribute__((packed)) MotorData_t;

extern MotorData_t Motor[MAX_MOTORS];

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
}  __attribute__((packed)) config_mkk_eeprom_t;

extern config_mkk_eeprom_t config_mkk_eeprom;
*/

uint8_t config_mkk_crc(uint8_t offset)
{
	uint8_t crc = 0xaa;
	for(int i=offset; i<(offset+7); i++)
	{
		crc += config_mkk.trans.buf[i];
	}
	return crc;
}


MotorData_t Motor[MAX_MOTORS];
config_mkk_eeprom_t config_mkk_eeprom;


#define BL_READMODE_CONFIG	       16

#define CONFIG_MKK_EEPROM_REVISION          2

#define MASK_SET_PWM_SCALING       0x01
#define MASK_SET_CURRENT_LIMIT     0x02
#define MASK_SET_TEMP_LIMIT        0x04
#define MASK_SET_CURRENT_SCALING   0x08
#define MASK_SET_BITCONFIG         0x10
#define MASK_RESET_CAPCOUNTER      0x20
#define MASK_SET_DEFAULT_PARAMS    0x40
#define MASK_SET_SAVE_EEPROM       0x80

#define BITCONF_REVERSE_ROTATION   0x01


void init_config_mkk(void)
{
  config_mkk.nb_err = 0;
  config_mkk.read_config = 0;

  config_mkk.trans.status = I2CTransSuccess;
  
  for(int i=0; i < MAX_MOTORS; i++)
  {
    Motor[i].Version     = 0;
    Motor[i].Current     = 0;
    Motor[i].MaxPWM      = 0;
    Motor[i].Temperature = 0;
  }
}

#include "subsystems/actuators/actuators_mkk2.h"

void periodic_config_mkk_read_status(void)
{
  static int read_nr = 0;

  if (!actuators_mkk2.actuators_delay_done)
    return;

  switch (config_mkk.trans.status) {
    case I2CTransFailed:
      config_mkk.nb_err++;
      break;
    case I2CTransSuccess:
    case I2CTransDone:
      if (config_mkk.trans.len_r == 3)
      {
          Motor[read_nr].Current = config_mkk.trans.buf[0];
          Motor[read_nr].MaxPWM = config_mkk.trans.buf[1];
          Motor[read_nr].Temperature = config_mkk.trans.buf[2];
      }
      else if (config_mkk.trans.len_r == 8)
      {
        config_mkk_parse_eeprom();
      }
      break;
    default:
      config_mkk.nb_err++;
      return;
  }

  // Read Config
  if (config_mkk.read_config > 0)
  {
    config_mkk.read_config = 0;
    config_mkk_read_eeprom();


    i2c_submit(&ACTUATORS_MKK2_DEVICE, &config_mkk.trans);
  }
  // Read Status
  else
  {
    read_nr++;
    if (read_nr >= MAX_MOTORS)
      read_nr = 0;
    const uint8_t actuators_addr[ACTUATORS_MKK2_NB] = ACTUATORS_MKK2_ADDR;
    config_mkk.trans.type = I2CTransRx;
    config_mkk.trans.len_r = 3;
    config_mkk.trans.slave_addr = actuators_addr[read_nr];
  }  


}

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"


void periodic_config_mkk_telemetry(void)
{
    static uint8_t send_nr = 0;

    DOWNLINK_SEND_MKK(DefaultChannel, DefaultDevice, &send_nr, &Motor[send_nr].MaxPWM, &Motor[send_nr].Current, &Motor[send_nr].Temperature);

    send_nr++;
    if (send_nr >= MAX_MOTORS)
      send_nr = 0;
}


#define RETURN_IF_NOT_KILLMODE() {}

void config_mkk_read_eeprom(void)
{
    // Do not read config while running
    RETURN_IF_NOT_KILLMODE();

    // New I2C Write/Read Transaction
    config_mkk.trans.type = I2CTransTxRx;
    config_mkk.trans.slave_addr = 0x52 + config_mkk.addr * 2;
    config_mkk.trans.len_w = 2;
    config_mkk.trans.buf[0] = 0;
    config_mkk.trans.buf[1] = (BL_READMODE_CONFIG<<3);
    config_mkk.trans.len_r = 8;
}

void config_mkk_parse_eeprom(void)
{
    config_mkk_eeprom.crc            = config_mkk.trans.buf[7];   // checksum
    if (config_mkk_crc(0) != config_mkk_eeprom.crc)
    {
        config_mkk.nb_err++;
    }
    else
    {
        config_mkk_eeprom.revision       = config_mkk.trans.buf[0];   // must be BL_revision
        config_mkk_eeprom.SetMask        = config_mkk.trans.buf[1];   // settings mask
        config_mkk_eeprom.PwmScaling     = config_mkk.trans.buf[2];   // maximum value of control pwm, acts like a thrust limit
        config_mkk_eeprom.CurrentLimit   = config_mkk.trans.buf[3];   // current limit in A
        config_mkk_eeprom.TempLimit      = config_mkk.trans.buf[4];   // in °C
        config_mkk_eeprom.CurrentScaling = config_mkk.trans.buf[5];   // scaling factor for current measurement
        config_mkk_eeprom.BitConfig      = config_mkk.trans.buf[6];   // see defines above
    }
}

void config_mkk_send_eeprom(void)
{
    // Do not upload while running
    RETURN_IF_NOT_KILLMODE();

    // Do not upload bad data:
    if (config_mkk_eeprom.revision != CONFIG_MKK_EEPROM_REVISION)
      return;

    // New I2C Write Transaction
    config_mkk.trans.type = I2CTransTx;
    config_mkk.trans.slave_addr = 0x52 + config_mkk.addr * 2;
    config_mkk.trans.len_w = 10;
    config_mkk.trans.buf[0] = 0;
    config_mkk.trans.buf[1] = (BL_READMODE_CONFIG<<3);

    config_mkk.trans.buf[2] = config_mkk_eeprom.revision;
    config_mkk.trans.buf[3] = config_mkk_eeprom.SetMask;
    config_mkk.trans.buf[4] = config_mkk_eeprom.PwmScaling;
    config_mkk.trans.buf[5] = config_mkk_eeprom.CurrentLimit;
    config_mkk.trans.buf[6] = config_mkk_eeprom.TempLimit;
    config_mkk.trans.buf[7] = config_mkk_eeprom.CurrentScaling;
    config_mkk.trans.buf[8] = config_mkk_eeprom.BitConfig;
    config_mkk.trans.buf[9] = config_mkk_crc(2);

    i2c_submit(&ACTUATORS_MKK2_DEVICE, &config_mkk.trans);

}

