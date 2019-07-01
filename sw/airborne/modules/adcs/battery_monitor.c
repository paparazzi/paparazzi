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

/** @file modules/adcs/battery_monitor.c
 * driver for ADC AD7997 on a custom made power board version 4.0 and 5.0
 *
 */
#include "modules/adcs/battery_monitor.h"

#if BATTERY_MONITOR_REV4
PRINT_CONFIG_MSG("Battery monitor: using older revision 4");
#endif /* BATTERY_MONITOR_REV4 */

struct BatMonBus batmonbus;
struct BatMonBal batmonbal1;
struct BatMonBal batmonbal2;

// can be tuned via datalink
int16_t batmon_current_offset;
float batmon_current_sensitivity;
int16_t batmon_temp_offset;
float batmon_temp_sensitivity;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_batmon(struct transport_tx *trans, struct link_device *dev)
{
  static uint16_t power_status;
#if BATTERY_MONITOR_REV4
  /* revision 4 */
  static uint8_t version = 1;
  power_status = 0; // no status
#else
  /* revision 5 */
  static uint8_t version = 2;
  power_status = batmonbus.bus_tempsensors_mvolts[4]; // VIN7 has position 5 in temp map, hence index 4
#endif

  uint8_t batmonbus_bus_trans_status = batmonbus.bus_trans.status;
  uint8_t batmonbal1_bus_trans_status = batmonbal1.bus_trans.status;
  uint8_t batmonbal2_bus_trans_status = batmonbal2.bus_trans.status;
  pprz_msg_send_BATTERY_MONITOR(trans, dev, AC_ID,
      &version,
      &batmonbus.bus_status,
      &batmonbus_bus_trans_status,
      &batmonbus.bus_current_mvolts,
      &batmonbus.bus_current,
      &batmonbus.bus_voltage_mvolts,
      TEMP_SENSORS_NB,
      batmonbus.bus_tempsensors_mvolts,
      TEMP_SENSORS_NB,
      batmonbus.bus_temp,
      &batmonbal1_bus_trans_status,
      BATTERY_CELLS_NB,
      batmonbal1.bat_cell_mvolts,
      &batmonbal2_bus_trans_status,
      BATTERY_CELLS_NB,
      batmonbal2.bat_cell_mvolts,
      &power_status);
}
#endif

/**
 *
 * Translates the channel number to the value
 * for address pointer register
 * Channels are numbered from 1 to 8
 * @param chan - desired channel
 * @return addres pointer Byte (see AD7997/8 datasheet)
 */
uint8_t battery_monitor_get_address(uint8_t chan) {
  if ((chan>8) || (chan<1)){ // sanity check
    return 0x80;
  }
  static uint8_t adr[] = {0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0};
  return adr[chan-1];
}

/**
 * Initializes bus ADC
 */
void battery_monitor_init_bus(void){
  // can be tuned via datalink
  batmon_current_offset = BATTERY_MONITOR_CURRENT_OFFSET;
  batmon_current_sensitivity = BATTERY_MONITOR_CURRENT_SENSITIVITY;
  batmon_temp_offset = BATTERY_MONITOR_TEMP_OFFSET;
  batmon_temp_sensitivity = BATTERY_MONITOR_TEMP_SENSITIVITY;

  // init Bus ADC
  // transactions
  batmonbus.bus_trans.status = I2CTransDone;
  batmonbus.addr = BATTERY_MONITOR_BUS_ADC_I2C_ADDR;
  batmonbus.bus_status = BATTERY_MONITOR_BUS_CURRENT_REQ; // device status
  // Current readings
  batmonbus.bus_current_mvolts = 0; // mV
  batmonbus.bus_current = 0; // A
  // Bus voltage readings
  batmonbus.bus_voltage_mvolts = 0; //mV
  // Temperature readings
  batmonbus.bus_brd_tmp = 0; // C
  memset(batmonbus.bus_tempsensors_mvolts, 0, sizeof(batmonbus.bus_tempsensors_mvolts));
  memset(batmonbus.bus_temp, 0, sizeof(batmonbus.bus_temp));
  batmonbus.t_idx = 0; // temp sensor index
  batmonbus.data = 0;
}

/**
 * Initalizes balance ADC
 */
void battery_monitor_init_balance(struct BatMonBal* batmonbal){
  batmonbal->bus_trans.status = I2CTransDone;
  batmonbal->cell_index = 0;
  memset(batmonbal->bat_cell_mvolts, 0, sizeof(batmonbal->bat_cell_mvolts));
  batmonbal->data = 0;
}

/**
 * Init variables
 */
void battery_monitor_init(void) {
  // bus ADC
  battery_monitor_init_bus();

  // balance 1 ADC
  battery_monitor_init_balance(&batmonbal1);
  batmonbal1.addr = BATTERY_MONITOR_BALANCE_BAT1_ADC_I2C_ADDR;

  // balance 2 ADC
  battery_monitor_init_balance(&batmonbal2);
  batmonbal2.addr = BATTERY_MONITOR_BALANCE_BAT2_ADC_I2C_ADDR;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_BATTERY_MONITOR, send_batmon);
#endif
}

/**
 * Event function
 * Check i2c transaction status for each device
 */
void battery_monitor_event(void){
  battery_monitor_check_i2c_transaction(&batmonbal1.bus_trans);
  battery_monitor_check_i2c_transaction(&batmonbal2.bus_trans);
  battery_monitor_check_i2c_transaction(&batmonbus.bus_trans);
}

/**
 * Complete i2c transactions once they succeed or fail
 */
void battery_monitor_check_i2c_transaction(struct i2c_transaction* t){
  switch (t->status) {
    case I2CTransPending:
      // wait and do nothing
      break;
    case I2CTransRunning:
      // wait and do nothing
      break;
    case I2CTransSuccess:
      // set to done
      t->status = I2CTransDone;
      break;
    case I2CTransFailed:
      // set to done
      t->status = I2CTransDone;
      break;
    case I2CTransDone:
      // do nothing
      break;
    default:
      break;
  }
}

/**
 * Read bus (current, voltage and temperature sensors)
 */
void battery_monitor_read_bus(void){
  batmonbus.data = 0; // erase at each iteration

  switch (batmonbus.bus_status) {
    // reqtest BUS_CURRENT measurement
    case BATTERY_MONITOR_BUS_CURRENT_REQ:
      // set ADC to current channel
      batmonbus.bus_trans.buf[0] = battery_monitor_get_address(
          (uint8_t)BATTERY_MONITOR_BUS_CURRENT_CHANNEL);
      //set to zero so we can detect an error
      batmonbus.bus_current = 0;

      // non-blocking transaction
      if (i2c_transceive(&BATTERY_MONITOR_I2C_DEV, &batmonbus.bus_trans, batmonbus.addr, 1, 2)){
        batmonbus.bus_status = BATTERY_MONITOR_BUS_CURRENT_READ;
      }
      break;

      // read BUS_CURRENT data
    case BATTERY_MONITOR_BUS_CURRENT_READ:
      if (batmonbus.bus_trans.status == I2CTransDone) {
        // read data
        batmonbus.data = (uint16_t) (batmonbus.bus_trans.buf[0] << 8 | batmonbus.bus_trans.buf[1]);
        // NOTE: we are not using the ALERT_FLAG at the moment,
        // get counts
        batmonbus.bus_current_mvolts = batmonbus.data & 0xFFF;
        // shift right by 2 bits if 10-bit reads only
        if (BATTERY_MONITOR_BIT_RES == 10){
          batmonbus.bus_current_mvolts = (batmonbus.bus_current_mvolts) >> 2;
        }
        // convert to mV
        batmonbus.bus_current_mvolts = (uint16_t)(
            (float)batmonbus.bus_current_mvolts * BATTERY_MONITOR_VREF_MULT);
        // convert to [A]
        batmonbus.bus_current = ((float)batmonbus.bus_current_mvolts +
            (float)batmon_current_offset) / batmon_current_sensitivity;
        //update electrical subsystem
        electrical.current = batmonbus.bus_current;

        // increment status
        batmonbus.bus_status = BATTERY_MONITOR_BUS_VOLTAGE_REQ;
      }
      break;

      // request voltage data
    case BATTERY_MONITOR_BUS_VOLTAGE_REQ:
      // set ADC to voltage channel
      batmonbus.bus_trans.buf[0] = battery_monitor_get_address(
          (uint8_t)BATTERY_MONITOR_BUS_VOLTAGE_CHANNEL);
      //set to zero so we can detect an error
      batmonbus.bus_voltage_mvolts = 0;

      // non-blocking transaction
      if (i2c_transceive(&BATTERY_MONITOR_I2C_DEV, &batmonbus.bus_trans, batmonbus.addr, 1, 2)){
        batmonbus.bus_status = BATTERY_MONITOR_BUS_VOLTAGE_READ;
      }
      break;

      // read voltage data
    case BATTERY_MONITOR_BUS_VOLTAGE_READ:
      if (batmonbus.bus_trans.status == I2CTransDone) {
        // read data
        batmonbus.data = (uint16_t) (batmonbus.bus_trans.buf[0] << 8 | batmonbus.bus_trans.buf[1]);
        // NOTE: we are not using the ALERT_FLAG at the moment,
        // get counts
        batmonbus.bus_voltage_mvolts = batmonbus.data & 0xFFF;
        // shift right by 2 bits if 10-bit reads only
        if (BATTERY_MONITOR_BIT_RES == 10){
          batmonbus.bus_voltage_mvolts = (batmonbus.bus_voltage_mvolts) >> 2;
        }
        // convert to mV
        batmonbus.bus_voltage_mvolts = (uint16_t)(
            (float)batmonbus.bus_voltage_mvolts * BATTERY_MONITOR_VREF_MULT);
        // convert to actual voltage
        batmonbus.bus_voltage_mvolts = (uint16_t)(
            (float)batmonbus.bus_voltage_mvolts * BatmonVbusGain);

        //update electrical subsystem
        if (batmonbus.bus_voltage_mvolts != 0) {
          electrical.vsupply = (float)(batmonbus.bus_voltage_mvolts) / 1000.f;
        }
        else {
          electrical.vsupply = 0.f;
        }

        // update status
        batmonbus.bus_status = BATTERY_MONITOR_BUS_TEMPERATURE_REQ;
      }
      break;

      // request temperature data
    case BATTERY_MONITOR_BUS_TEMPERATURE_REQ:
      // set ADC to correct temperature channel
      batmonbus.bus_trans.buf[0] = battery_monitor_get_address(
          battery_monitor_tempmap[batmonbus.t_idx]);
      //set to zero so we can detect an error
      batmonbus.bus_temp[batmonbus.t_idx] = 0;

      // non-blocking transaction
      if (i2c_transceive(&BATTERY_MONITOR_I2C_DEV, &batmonbus.bus_trans, batmonbus.addr, 1, 2)){
        batmonbus.bus_status = BATTERY_MONITOR_BUS_TEMPERATURE_READ;
      }
      break;

      // read temperature data
    case BATTERY_MONITOR_BUS_TEMPERATURE_READ:
      if (batmonbus.bus_trans.status == I2CTransDone) {
        // read data
        batmonbus.data = (uint16_t) (batmonbus.bus_trans.buf[0] << 8 | batmonbus.bus_trans.buf[1]);
        // NOTE: we are not using the ALERT_FLAG at the moment,
        // get counts
        batmonbus.bus_tempsensors_mvolts[batmonbus.t_idx] = batmonbus.data & 0xFFF;
        // shift right by 2 bits if 10-bit reads only
        if (BATTERY_MONITOR_BIT_RES == 10){
          batmonbus.bus_tempsensors_mvolts[batmonbus.t_idx] =
              (batmonbus.bus_tempsensors_mvolts[batmonbus.t_idx]) >> 2;
        }
        // convert to mV
        batmonbus.bus_tempsensors_mvolts[batmonbus.t_idx] = (uint16_t)(
            (float)batmonbus.bus_tempsensors_mvolts[batmonbus.t_idx] * BATTERY_MONITOR_VREF_MULT);
        // convert to temperature[C]
        batmonbus.bus_temp[batmonbus.t_idx] =
            ((float)batmonbus.bus_tempsensors_mvolts[batmonbus.t_idx] +
                (float)batmon_temp_offset ) / batmon_temp_sensitivity;
        // increment counter
        batmonbus.t_idx++;
        if (batmonbus.t_idx == TEMP_SENSORS_NB) {
          batmonbus.t_idx = 0;
          batmonbus.bus_status = BATTERY_MONITOR_BUS_CURRENT_REQ; // to loop through
        }
      }
      break;
    default:
      // a recovery in case of a glitch
      batmonbus.bus_status = BATTERY_MONITOR_BUS_CURRENT_REQ;
      break;
  }
}


/**
 * Read Balance ADC 1
 */
void battery_monitor_read_balance_ports_1(void) {
  battery_monitor_read_balance_ports(&batmonbal1);
}

/**
 * Read Balance ADC 2
 */
void battery_monitor_read_balance_ports_2(void) {
  battery_monitor_read_balance_ports(&batmonbal2);
}

/**
 * Read balance ADC
 */
void battery_monitor_read_balance_ports(struct BatMonBal* batmonbal) {
  // non-blocking transaction
  // if transaction is done, request another measurement
  // NOTE: optionally add startup delay, but it shouldn't be necessary
  if (batmonbal->bus_trans.status == I2CTransDone) {
    // read data first (worst case we get some zeros)
    batmonbal->data = (uint16_t) (batmonbal->bus_trans.buf[0] << 8 | batmonbal->bus_trans.buf[1]);
    // NOTE: we are not using the ALERT_FLAG at the moment,
    // get counts
    batmonbal->bat_cell_mvolts[batmonbal->cell_index] = batmonbal->data & 0xFFF;
    // shift right by 2 bits if 10-bit reads only
    if (BATTERY_MONITOR_BIT_RES == 10){
      batmonbal->bat_cell_mvolts[batmonbal->cell_index] =
          (batmonbal->bat_cell_mvolts[batmonbal->cell_index]) >> 2;
    }
    // convert to mV
    batmonbal->bat_cell_mvolts[batmonbal->cell_index] =
        (uint16_t)((float)batmonbal->bat_cell_mvolts[batmonbal->cell_index] * BATTERY_MONITOR_VREF_MULT);
    // convert to actual voltage
    // we get the multiplier from battery_monitor_cellgains array, which contains multipliers
    // for cells 1-6(in this order) battery_monitor_cellgains[0] gives us multiplier for cell_1
    // and so on
    batmonbal->bat_cell_mvolts[batmonbal->cell_index] =
        (uint16_t)((float)batmonbal->bat_cell_mvolts[batmonbal->cell_index] *
            battery_monitor_cellgains[batmonbal->cell_index]);

    // increment counter
    batmonbal->cell_index++;
    if (batmonbal->cell_index == BATTERY_CELLS_NB) {
      batmonbal->cell_index = 0;
    }

    // erase data
    batmonbal->data = 0; // erase at each iteration

    //set to zero so we can detect an error
    batmonbal->bat_cell_mvolts[batmonbal->cell_index] = 0;

    // set the right address into ADC
    // cell_index is the number of the cell we want to probe
    // battery_monitor_cellmap1 contains the mapping of channels for given cell number
    // i.e. Cell_1 (cell_idx = 0) has channel 2
    // this gets translated into hexadecimal number representing the channel internally
    batmonbal->bus_trans.buf[0] =
        battery_monitor_get_address((uint8_t)battery_monitor_cellmap[batmonbal->cell_index]);

    // request more data
    i2c_transceive(&BATTERY_MONITOR_I2C_DEV, &batmonbal->bus_trans, batmonbal->addr, 1, 2);
  }
}
