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
 * Graupner HOTT general air module description.
 */

#ifndef HOTT_GAM_H
#define HOTT_GAM_H

#include "autopilot.h"

//Graupner #33611 General Air Module
#define HOTT_TELEMETRY_GAM_SENSOR_ID  0x8D

struct HOTT_GAM_MSG {
  int8_t start_byte;          //#01 start int8_t constant value 0x7c
  int8_t gam_sensor_id;       //#02 EAM sensort id. constat value 0x8d
  int8_t warning_beeps;        //#03 1=A 2=B ... 0x1a=Z  0 = no alarm
  // Q  Min cell voltage sensor 1
  // R  Min Battery 1 voltage sensor 1
  // J  Max Battery 1 voltage sensor 1
  // F  Min temperature sensor 1
  // H  Max temperature sensor 1
  // S  Min Battery 2 voltage sensor 2
  // K  Max Battery 2 voltage sensor 2
  // G  Min temperature sensor 2
  // I  Max temperature sensor 2
  // W  Max current
  // V  Max capacity mAh
  // P  Min main power voltage
  // X  Max main power voltage
  // O  Min altitude
  // Z  Max altitude
  // C  negative difference m/s too high
  // A  negative difference m/3s too high
  // N  positive difference m/s too high
  // L  positive difference m/3s too high
  // T  Minimum RPM
  // Y  Maximum RPM

  int8_t sensor_id;            //#04 constant value 0xd0
  int8_t alarm_invers1;        //#05 alarm bitmask. Value is displayed inverted
  //Bit#  Alarm field
  // 0  all cell voltage
  // 1  Battery 1
  // 2  Battery 2
  // 3  Temperature 1
  // 4  Temperature 2
  // 5  Fuel
  // 6  mAh
  // 7  Altitude
  int8_t alarm_invers2;        //#06 alarm bitmask. Value is displayed inverted
  //Bit#  Alarm Field
  // 0  main power current
  // 1  main power voltage
  // 2  Altitude
  // 3  m/s
  // 4  m/3s
  // 5  unknown
  // 6  unknown
  // 7  "ON" sign/text msg active

  int8_t cell1;                //#07 cell 1 voltage lower value. 0.02V steps, 124=2.48V
  int8_t cell2;                //#08
  int8_t cell3;                //#09
  int8_t cell4;                //#10
  int8_t cell5;                //#11
  int8_t cell6;                //#12
  int8_t batt1_L;              //#13 battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V
  int8_t batt1_H;              //#14
  int8_t batt2_L;              //#15 battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V
  int8_t batt2_H;              //#16
  int8_t temperature1;        //#17 temperature 1. offset of 20. a value of 20 = 0°C
  int8_t temperature2;        //#18 temperature 2. offset of 20. a value of 20 = 0°C
  int8_t fuel_procent;        //#19 Fuel capacity in %. Values 0--100
  // graphical display ranges: 0-25% 50% 75% 100%
  int8_t fuel_ml_L;            //#20 Fuel in ml scale. Full = 65535!
  int8_t fuel_ml_H;            //#21
  int8_t rpm_L;                //#22 RPM in 10 RPM steps. 300 = 3000rpm
  int8_t rpm_H;                //#23
  int8_t altitude_L;          //#24 altitude in meters. offset of 500, 500 = 0m
  int8_t altitude_H;          //#25
  int8_t climbrate_L;          //#26 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
  int8_t climbrate_H;          //#27
  int8_t climbrate3s;          //#28 climb rate in m/3sec. Value of 120 = 0m/3sec
  int8_t current_L;            //#29 current in 0.1A steps
  int8_t current_H;            //#30
  int8_t main_voltage_L;      //#31 Main power voltage using 0.1V steps
  int8_t main_voltage_H;      //#32
  int8_t batt_cap_L;          //#33 used battery capacity in 10mAh steps
  int8_t batt_cap_H;          //#34
  int8_t speed_L;              //#35 (air?) speed in km/h(?) we are using ground speed here per default
  int8_t speed_H;              //#36
  int8_t min_cell_volt;        //#37 minimum cell voltage in 2mV steps. 124 = 2,48V
  int8_t min_cell_volt_num;    //#38 number of the cell with the lowest voltage
  int8_t rpm2_L;              //#39 RPM in 10 RPM steps. 300 = 3000rpm
  int8_t rpm2_H;              //#40
  int8_t general_error_number;  //#41 Voice error == 12. TODO: more docu
  int8_t pressure;            //#42 Pressure up to 16bar. 0,1bar scale. 20 = 2bar
  int8_t version;              //#43 version number TODO: more info?
  int8_t stop_byte;            //#44 stop int8_t
  //#45 CRC/Parity
};

static void hott_init_gam_msg(struct HOTT_GAM_MSG *hott_gam_msg)
{
  memset(hott_gam_msg, 0, sizeof(struct HOTT_GAM_MSG));
  hott_gam_msg->start_byte = 0x7C;
  hott_gam_msg->gam_sensor_id = HOTT_TELEMETRY_GAM_SENSOR_ID;
  hott_gam_msg->sensor_id = 0xD0;
  hott_gam_msg->stop_byte = 0x7D;
}

static void hott_update_gam_msg(struct HOTT_GAM_MSG *hott_gam_msg)
{
  hott_gam_msg->temperature1 = 1;
  hott_gam_msg->temperature2 = 2; // 0°C
  hott_gam_msg->altitude_L = 3;
  hott_gam_msg->climbrate_L = 4;
  hott_gam_msg->climbrate3s = 5;  // 0 m/3s using filtered data here
  hott_gam_msg->current_L = 6;
  hott_gam_msg->main_voltage_L = 7;
  hott_gam_msg->batt_cap_L = 8;
  hott_gam_msg->fuel_procent = 9;  // my fuel are electrons :)
  hott_gam_msg->speed_L = 10;

  //display ON when motors are armed
  if (autopilot_motors_on) {
    hott_gam_msg->alarm_invers2 |= 0x80;
  } else {
    hott_gam_msg->alarm_invers2 &= 0x7f;
  }
}

#endif /* HOTT_GAM_H */
