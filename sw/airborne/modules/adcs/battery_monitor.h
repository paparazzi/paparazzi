/*
 * Copyright (C) 2008-2015 The Paparazzi Team
 * 2017, Utah State University, http://aggieair.usu.edu/
 * Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
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

/** @file modules/adcs/battery_monitor.h
 * driver for ADC AD7997 on a custom made power board version 4.0 and 5.0
 *
 * Power board has:
 *
 * BUS_ADC
 * two possible addresses:
 * 010 0001 (AD7997-0) = 0x21 (pprz: 0x42)
 * 010 0011 (AD7997-1) = 0x23 (pprz: 0x46)
 * channels:
 * VIN1 - Temp_1
 * VIN2 - Current sensor
 * VIN3 - PWR_BRD_Temp
 * VIN4 - Vin
 * VIN5 - Temp_2
 * VIN6 - Batt_Temp1
 * VIN7 - Temp_3
 * VIN8 - Batt_temp_2
 *
 * BALANCE_ADC_1 (battery 1)
 * address:
 * 010 0000 (AD7997-1 or AD7997-0) = 0x20 (pprz: 0x40)
 * channels:
 * VIN1 - NC
 * VIN2 - Cell_1
 * VIN3 - NC
 * VIN4 - Cell_2
 * VIN5 - Cell_5
 * VIN6 - Cell_3
 * VIN7 - Cell_6
 * VIN8 - Cell_4
 *
 * BALANCE_ADC_2 (battery 2)
 * address:
 * 010 0010 (AD7997-0) = 0x22 (pprz: 0x44)
 * 010 0100 (AD7997-1) = 0x24 (pprz: 0x48)
 * channels:
 * VIN1 - NC
 * VIN2 - Cell_1
 * VIN3 - NC
 * VIN4 - Cell_2
 * VIN5 - NC
 * VIN6 - Cell_3
 * VIN7 - NC
 * VIN8 - Cell_4
 */
#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"
#include "modules/energy/electrical.h"

// revision information
#if BATTERY_MONITOR_REV4
/* revision 4 */
#define BATTERY_MONITOR_BIT_RES 10 // define bit resolution: 12 for AD7998 (rev4)
#else
/* revision 5 */
#define BATTERY_MONITOR_BIT_RES 12 // define bit resolution: 10 for AD7997 (rev5)
#endif /* BATTERY_MONITOR_REV4 */

// default reference (5 Volts)
#define BATTERY_MONITOR_VREF 5000 //mV

// reading multiplier
#define BATTERY_MONITOR_VREF_MULT  BATTERY_MONITOR_VREF/(1<<BATTERY_MONITOR_BIT_RES)

// we are using 6 cells by default
#define BATTERY_CELLS_NB 6

// we are using 6 temp sensors right now
#define TEMP_SENSORS_NB 6

/*
 *  cell ADC gains
 * representing inverse values of the voltage dividers
 */
#if BATTERY_MONITOR_REV4
/* revision 4 */
#define GAIN_CELL_1 1.001
#define GAIN_CELL_2 2.011
#define GAIN_CELL_3 3.027
#define GAIN_CELL_4 4.063
#define GAIN_CELL_5 4.217
#define GAIN_CELL_6 4.973
#else
/* revision 5 */
#define GAIN_CELL_1 1.0
#define GAIN_CELL_2 2.0
#define GAIN_CELL_3 3.0
#define GAIN_CELL_4 4.03
#define GAIN_CELL_5 5.0
#define GAIN_CELL_6 6.0
#endif /* BATTERY_MONITOR_REV4 */
static const float battery_monitor_cellgains[] = {GAIN_CELL_1, GAIN_CELL_2,
    GAIN_CELL_3, GAIN_CELL_4, GAIN_CELL_5, GAIN_CELL_6};

/**
 * Cell map - which cell is which channel
 * Channels are 1-indexed, ie.e. Channel 1 - Channel 8
 * Cells are 0-indexed (because of their position in the array)
 * So cell at battery_monitor_cellmap[0] is the first cell etc.
 */
static const uint8_t battery_monitor_cellmap[] = {2,4,6,8,5,7}; // 6s cellmap (battery1)
// for 4 cell battery the last two readings should be zero (channel tied down to GND)
// or HIGH (channel tied to VCC)

// define channels
#define BATTERY_MONITOR_BUS_CURRENT_CHANNEL 2
#define BATTERY_MONITOR_BUS_VOLTAGE_CHANNEL 4

/*
 * Channels for the temp sensors
 */
static const uint8_t battery_monitor_tempmap[] = {1,3,5,6,7,8};

/*
 * Define bus voltage gain based on
 * values of voltage divider at
 * power board
 */
#if BATTERY_MONITOR_REV4
/* revision 4 */
static const float BatmonVbusGain = 4.953980661;
#else
/* revision 5 */
static const float BatmonVbusGain = 6.0;
#endif /* BATTERY_MONITOR_REV4 */

/*
 * Define current sensor calibration
 * include default values
 * See datasheet for your current sensor
 * to get the typical values
 */
#ifndef BATTERY_MONITOR_CURRENT_OFFSET //mV
#ifdef BATTERY_MONITOR_REV4
/* revision 4 */
#define BATTERY_MONITOR_CURRENT_OFFSET -120
#else
/* revision 5 */
#define BATTERY_MONITOR_CURRENT_OFFSET -657
#endif /* BATTERY_MONITOR_REV4 */
#endif /* BATTERY_MONITOR_CURRENT_OFFSET */

#ifndef BATTERY_MONITOR_CURRENT_SENSITIVITY //mV/A
#define BATTERY_MONITOR_CURRENT_SENSITIVITY 25.6
#endif

/*
 * Calibration for the temperature sensors
 * (for now just one for all of them)
 * values for TMP35
 */
#ifndef BATTERY_MONITOR_TEMP_OFFSET //mV
#define BATTERY_MONITOR_TEMP_OFFSET 250
#endif
#ifndef BATTERY_MONITOR_TEMP_SENSITIVITY //mV/C
#define BATTERY_MONITOR_TEMP_SENSITIVITY 10
#endif

// can be tuned via datalink
extern int16_t batmon_current_offset;
extern float batmon_current_sensitivity;
extern int16_t batmon_temp_offset;
extern float batmon_temp_sensitivity;


/**
 * Status for Bus ADC
 */
enum BatmonBusStatus {
  BATTERY_MONITOR_BUS_CURRENT_REQ,
  BATTERY_MONITOR_BUS_CURRENT_READ,
  BATTERY_MONITOR_BUS_VOLTAGE_REQ,
  BATTERY_MONITOR_BUS_VOLTAGE_READ,
  BATTERY_MONITOR_BUS_TEMPERATURE_REQ,
  BATTERY_MONITOR_BUS_TEMPERATURE_READ
};

/**
 * Battery monitor
 * Bus ADC struct
 */
struct BatMonBus {
  struct i2c_transaction bus_trans; // transaction
  enum BatmonBusStatus bus_status; // device status
  uint16_t data; // raw 16bit read from adc
  uint8_t addr; // i2c device address

  // Current readings
  uint16_t bus_current_mvolts; // mV
  float bus_current; // A

  // Bus voltage readings
  uint16_t bus_voltage_mvolts; //mV

  // Temperature readings
  float bus_brd_tmp; // C
  uint16_t bus_tempsensors_mvolts[TEMP_SENSORS_NB]; // mV
  float bus_temp[TEMP_SENSORS_NB]; // C
  uint8_t t_idx; // temp sensor index

};

/**
 * Battery monitor
 * Balance ADC struct
 */
struct BatMonBal {
  struct i2c_transaction bus_trans; // transaction
  uint16_t data; // raw 16bit read from adc
  uint8_t addr; // i2c device address

  //Balance ADC data
  uint16_t bat_cell_mvolts[BATTERY_CELLS_NB]; // mV

  // index for reading cells
  uint8_t cell_index;
};

extern struct BatMonBus batmonbus;
extern struct BatMonBal batmonbal1;
extern struct BatMonBal batmonbal2;

void battery_monitor_init(void);
void battery_monitor_init_bus(void);
void battery_monitor_init_balance(struct BatMonBal *);

void battery_monitor_read_bus(void);
void battery_monitor_read_balance_ports_1(void);
void battery_monitor_read_balance_ports_2(void);
void battery_monitor_read_balance_ports(struct BatMonBal *);

void battery_monitor_event(void);
void battery_monitor_check_i2c_transaction(struct i2c_transaction* t);

uint8_t battery_monitor_get_address(uint8_t channel);

#endif /* BATTERY_MONITOR_H */
