/*
 * Copyright (C) 2013 Sergey Krukowski <softsr@yahoo.de>
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

/** @file hott_eam.h
 *
 * Graupner HOTT electric air module description.
 */

#ifndef HOTT_EAM_H
#define HOTT_EAM_H

#include "subsystems/electrical.h"
#include "subsystems/imu.h"
#include "autopilot.h"
#include "state.h"

//Graupner #33620 Electric Air Module
#define HOTT_TELEMETRY_EAM_SENSOR_ID  0x8E

struct HOTT_EAM_MSG {
  int8_t start_byte;            //#01 start int8_t
  int8_t eam_sensor_id;         //#02 EAM sensort id. constat value 0x8e
  int8_t warning_beeps;         //#03 1=A 2=B ... or 'A' - 0x40 = 1
  // Q  Min cell voltage sensor 1
  // R  Min Battery 1 voltage sensor 1
  // J  Max Battery 1 voltage sensor 1
  // F  Mim temperature sensor 1
  // H  Max temperature sensor 1
  // S  Min cell voltage sensor 2
  // K  Max cell voltage sensor 2
  // G  Min temperature sensor 2
  // I  Max temperature sensor 2
  // W  Max current
  // V  Max capacity mAh
  // P  Min main power voltage
  // X  Max main power voltage
  // O  Min altitude
  // Z  Max altitude
  // C  (negative) sink rate m/sec to high
  // B  (negative) sink rate m/3sec to high
  // N  climb rate m/sec to high
  // M  climb rate m/3sec to high

  int8_t sensor_id;             //#04 constant value 0xe0
  int8_t alarm_invers1;         //#05 alarm bitmask. Value is displayed inverted
  //Bit#  Alarm field
  // 0  mAh
  // 1  Battery 1
  // 2  Battery 2
  // 3  Temperature 1
  // 4  Temperature 2
  // 5  Altitude
  // 6  Current
  // 7  Main power voltage
  int8_t alarm_invers2;         //#06 alarm bitmask. Value is displayed inverted
  //Bit#  Alarm Field
  // 0  m/s
  // 1  m/3s
  // 2  Altitude (duplicate?)
  // 3  m/s  (duplicate?)
  // 4  m/3s (duplicate?)
  // 5  unknown/unused
  // 6  unknown/unused
  // 7  "ON" sign/text msg active

  int8_t cell1_L;              //#07 cell 1 voltage lower value. 0.02V steps, 124=2.48V
  int8_t cell2_L;              //#08
  int8_t cell3_L;              //#09
  int8_t cell4_L;              //#10
  int8_t cell5_L;              //#11
  int8_t cell6_L;              //#12
  int8_t cell7_L;              //#13
  int8_t cell1_H;              //#14 cell 1 voltage high value. 0.02V steps, 124=2.48V
  int8_t cell2_H;              //#15
  int8_t cell3_H;              //#16
  int8_t cell4_H;              //#17
  int8_t cell5_H;              //#18
  int8_t cell6_H;              //#19
  int8_t cell7_H;              //#20

  uint16_t batt1_voltage;      //#21 #22 battery 1 voltage 0.02V steps

  uint16_t batt2_voltage;      //#23 #24 battery 2 voltage 0.02V steps

  int8_t temp1;                //#25 Temperature sensor 1. 0°=20, 26°=46
  int8_t temp2;                //#26 temperature sensor 2

  uint16_t altitude;          //#27 #28 Attitude lower value. unit: meters. Value of 500 = 0m

  uint16_t current;           //#29 #30 Current in 0.1 steps

  uint16_t main_voltage;      //#30 #31 Main power voltage (drive) in 0.1V steps

  uint16_t batt_cap;          //#32 #33 used battery capacity in 10mAh steps

  uint16_t climbrate;         //#34 #35 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s

  uint8_t climbrate3s;        //#36 climbrate in m/3sec. Value of 120 = 0m/3sec

  int8_t rpm_L;               //#37 RPM. Steps: 10 U/min
  int8_t rpm_H;               //#38

  int8_t electric_min;        //#39 Electric minutes. Time does start, when motor current is > 3 A
  int8_t electric_sec;        //#40

  int8_t speed_L;             //#41 (air?) speed in km/h. Steps 1km/h
  int8_t speed_H;             //#42

  int8_t stop_byte;           //#43 stop int8_t
  //#44 CRC/Parity
};

static void hott_init_eam_msg(struct HOTT_EAM_MSG *hott_eam_msg)
{
  memset(hott_eam_msg, 0, sizeof(struct HOTT_EAM_MSG));
  hott_eam_msg->start_byte = 0x7C;
  hott_eam_msg->eam_sensor_id = HOTT_TELEMETRY_EAM_SENSOR_ID;
  hott_eam_msg->sensor_id = 0xE0;
  hott_eam_msg->stop_byte = 0x7D;
}

static void hott_update_eam_msg(struct HOTT_EAM_MSG *hott_eam_msg)
{

  hott_eam_msg->batt1_voltage = electrical.vsupply;
  hott_eam_msg->batt2_voltage = electrical.vsupply;
  //hott_eam_msg->temp1 = 20 + imu.temperature / 10;
  //hott_eam_msg->temp2 = 20 + imu.temperature / 10;
  hott_eam_msg->altitude = (uint16_t)(500 + (stateGetPositionEnu_i()->z) / (1 << INT32_POS_FRAC));
  hott_eam_msg->current = electrical.current / 100;
  hott_eam_msg->main_voltage = electrical.vsupply;
  hott_eam_msg->batt_cap = 0;
  uint16_t speed_buf = (uint16_t)(stateGetHorizontalSpeedNorm_i() * 36 / 10 / (1 << INT32_SPEED_FRAC));
  hott_eam_msg->speed_L = speed_buf && 0xFF;
  hott_eam_msg->speed_H = (speed_buf >> 8) && 0xFF;

  hott_eam_msg->climbrate = (uint16_t)(30000 + (stateGetSpeedEnu_i()->z) * 100 / (1 << INT32_SPEED_FRAC));
  hott_eam_msg->climbrate3s = (uint8_t)(120 + (stateGetSpeedEnu_i()->z) * 3 / (1 << INT32_SPEED_FRAC));

  //display ON when motors are armed
  if (autopilot_motors_on) {
    hott_eam_msg->alarm_invers2 |= 0x80;
  } else {
    hott_eam_msg->alarm_invers2 &= 0x7f;
  }
}

#endif /* HOTT_EAM_H */
