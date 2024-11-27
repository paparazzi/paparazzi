/*
 * Florian Sansou florian.sansou@enac.fr
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
 */

/**
 * @file peripherals/spa06.c
 * @brief Sensor driver for SPA06/SPL06 sensor
 *
 * 
 *
 */

#include "peripherals/spa06.h"

#define SPL06_PRESSURE_OVERSAMPLING SPL06_OVERSAMPLING_64X_P  
#define SPL06_TEMPERATURE_OVERSAMPLING SPL06_OVERSAMPLING_1X_T

/** local function to extract raw data from i2c/spi buffer
 *  and compute compensation with selected precision
 */
static void parse_sensor_data(struct spa06_t *spa, uint8_t *data);
static void parse_calib_data(struct spa06_t *spa, uint8_t *coef);
static void compensate_pressure(struct spa06_t *spa);
static bool spa06_config(struct spa06_t *spa);
static bool spa06_get_calib(struct spa06_t *spa);
static int32_t raw_value_scale_factor(uint8_t oversampling);
static void spa06_register_write(struct spa06_t *spa, uint8_t reg, uint8_t value);
static void spa06_register_read(struct spa06_t *spa, uint8_t reg, uint16_t size);
static int32_t getTwosComplement(uint32_t raw, uint8_t length);


/**
 * @brief Initialize the spa06 sensor instance
 * 
 * @param spa The structure containing the configuration of the spa06 instance
 */
void spa06_init(struct spa06_t *spa)
{

    spa->data_available = false;
    spa->initialized = false;
    spa->status = SPA06_STATUS_UNINIT;
    spa->reset = false;
    spa->config_idx = 0;
    spa->calib_idx = 0;

  /* SPI setup */
  if(spa->bus == SPA06_SPI) {
    spa->spi.trans.cpol = SPICpolIdleHigh;
    spa->spi.trans.cpha = SPICphaEdge2;
    spa->spi.trans.dss = SPIDss8bit;
    spa->spi.trans.bitorder = SPIMSBFirst;
    spa->spi.trans.cdiv = SPIDiv16;

    spa->spi.trans.select = SPISelectUnselect;
    spa->spi.trans.slave_idx = spa->spi.slave_idx;
    spa->spi.trans.output_length = 0;
    spa->spi.trans.input_length = 0;
    spa->spi.trans.before_cb = NULL;
    spa->spi.trans.after_cb = NULL;
    spa->spi.trans.input_buf = spa->spi.rx_buf;
    spa->spi.trans.output_buf = spa->spi.tx_buf;
    spa->spi.trans.status = SPITransDone;

    // in SPI read, the first byte is garbage because writing the register address
    spa->rx_buffer = &spa->spi.rx_buf[1];
    spa->tx_buffer = spa->spi.tx_buf;
    spa->rx_length = &spa->spi.trans.input_length;
  }
  /* I2C setup */
  else {

    spa->i2c.trans.slave_addr = spa->i2c.slave_addr; // slave address
    spa->i2c.trans.status = I2CTransDone;  // set initial status: Done

    spa->rx_buffer = (uint8_t *)spa->i2c.trans.buf;
    spa->tx_buffer = (uint8_t *)spa->i2c.trans.buf;
    spa->rx_length = &spa->i2c.trans.len_r;

  }
}

/**
 * @brief Should be called periodically to request sensor readings
 * - First detects the sensor using WHO_AM_I reading
 * - Configures the sensor according the users requested configuration
 * - Requests a sensor reading 
 * 
 * @param spa The spa06 instance
 */
void spa06_periodic(struct spa06_t *spa)
{

  /* Idle */
  if((spa->bus == SPA06_SPI && spa->spi.trans.status == SPITransDone) || 
     (spa->bus == SPA06_I2C && spa->i2c.trans.status == I2CTransDone)) {

      switch (spa->status) {
        case SPA06_STATUS_UNINIT:
          spa->data_available = false;
          spa->initialized = false;
          if(spa->reset == false){
            spa06_register_write(spa, SPL06_REG_RST, SPL06_RESET_BIT_SOFT_RST);
            spa->timer = get_sys_time_usec();
          }
          uint32_t diff_val = (get_sys_time_usec() - spa->timer);
          if(diff_val < 4e4){
            spa->status = SPA06_STATUS_UNINIT; //Stay in uninit state for 40ms after reset
            break;
          }
          spa->reset = false;
          spa->status = SPA06_STATUS_IDLE;
          break;

        case SPA06_STATUS_IDLE:
          /* Request WHO_AM_I */
          spa06_register_read(spa, SPL06_REG_CHIP_ID, 2);
          break;


        case SPA06_STATUS_INIT_OK:
          spa06_register_read(spa, SPL06_REG_MODE_AND_STATUS, 1);
          break;

        case SPA06_STATUS_GET_CALIB:
          // request calibration data
          if(spa06_get_calib(spa)){
            spa->status = SPA06_STATUS_CONFIGURE;
          }
          //process in spa06_event()
          break;

        case SPA06_STATUS_CONFIGURE:
          if(spa06_config(spa)) {
            spa->status = SPA06_STATUS_READ_STATUS_REG;
            spa->initialized = true;
          }
          break;

        case  SPA06_STATUS_READ_STATUS_REG:
          // READ THE STATUS BYTE
          spa06_register_read(spa, SPL06_REG_MODE_AND_STATUS, 1);
          break;

        case  SPA06_STATUS_READ_DATA_REGS:
          // READ ALL 6 DATA REGISTERS
          spa06_register_read(spa, SPL06_REG_PRESSURE_B2, SPL06_PRESSURE_LEN + SPL06_TEMPERATURE_LEN);
          break;

        default:
          break;
      }
    }
}

/**
 * @brief Should be called in the event thread
 * - Configures the sensor and reads the responses
 * - Parse and request the sensor data 
 * 
 * @param spa The spa06 instance
 */
void spa06_event(struct spa06_t *spa)
{
  /* Successful transfer */
  if((spa->bus == SPA06_SPI && spa->spi.trans.status == SPITransSuccess) || 
     (spa->bus == SPA06_I2C && spa->i2c.trans.status == I2CTransSuccess)) {
      switch (spa->status) {
        case SPA06_STATUS_UNINIT:
          spa->reset = true; //If the transaction success, we ask a reset
          break;

        case SPA06_STATUS_IDLE:
          /* WHO_AM_I */
          if(spa->rx_buffer[0] == SPA06_CHIP_ID) {
            spa->device = SPA06;
            spa->status = SPA06_STATUS_INIT_OK;
          } 
          else if (spa->rx_buffer[0] == SPL06_CHIP_ID)
          {
            spa->device = SPL06;
            spa->status = SPA06_STATUS_INIT_OK;
          }
          else {
            spa->status = SPA06_STATUS_IDLE;
          }
          break;
        
        case SPA06_STATUS_INIT_OK:
          uint8_t status = spa->rx_buffer[0];
          if((status & (SPL06_MEAS_CFG_COEFFS_RDY | SPL06_MEAS_CFG_SENSOR_RDY)) == (SPL06_MEAS_CFG_COEFFS_RDY | SPL06_MEAS_CFG_SENSOR_RDY) ){
            spa->status = SPA06_STATUS_GET_CALIB;
          }
          break;

        case SPA06_STATUS_GET_CALIB:
          // compute calib
          parse_calib_data(spa, &spa->rx_buffer[0]); 
          break;

        case SPA06_STATUS_CONFIGURE:
          if(spa06_config(spa)) {
            spa->status = SPA06_STATUS_READ_STATUS_REG;
            spa->initialized = true;
          }
          break;

        case SPA06_STATUS_READ_STATUS_REG:
          // check status byte
          if ((spa->rx_buffer[0] & (SPL06_MEAS_CFG_PRESSURE_RDY | SPL06_MEAS_CFG_TEMPERATURE_RDY)) == (SPL06_MEAS_CFG_PRESSURE_RDY | SPL06_MEAS_CFG_TEMPERATURE_RDY)) {
            spa->status = SPA06_STATUS_READ_DATA_REGS;
          }
          break;

        case SPA06_STATUS_READ_DATA_REGS:
          // parse sensor data, compensate temperature first, then pressure
          parse_sensor_data(spa, &spa->rx_buffer[0]);
          compensate_pressure(spa);
          spa->data_available = true;
          spa->status = SPA06_STATUS_READ_STATUS_REG;
          break;

        default:
          spa->status = SPA06_STATUS_GET_CALIB; // just to avoid the compiler's warning message
          break;
      }
      if(spa->bus == SPA06_I2C){
        spa->i2c.trans.status = I2CTransDone; 
      }
      else{
        spa->spi.trans.status = SPITransDone;
      }

    } else if ((spa->bus == SPA06_SPI && spa->spi.trans.status == SPITransFailed) || 
               (spa->bus == SPA06_I2C && spa->i2c.trans.status == I2CTransFailed)) {
      /* try again */
      if (!spa->initialized) {
        spa->status = SPA06_STATUS_UNINIT;
      }
      if(spa->bus == SPA06_I2C){
        spa->i2c.trans.status = I2CTransDone; 
      }
      else{
        spa->spi.trans.status = SPITransDone;
      }
    }

  return;
}

static void parse_sensor_data(struct spa06_t *spa, uint8_t *data)
{
  spa->raw_pressure = getTwosComplement((data[0] << 16) + (data[1] << 8) + data[2], 24);
  spa->raw_temperature = getTwosComplement((data[3] << 16) + (data[4] << 8) + data[5], 24);
}


/**
 *  @brief This internal API is used to parse the calibration data, compensates
 *  it and store it in device structure (float version)
 */
static void parse_calib_data(struct spa06_t *spa, uint8_t *coef)
{
  switch(spa->calib_idx) {
    case 0:
      // 0x11 c0 [3:0] + 0x10 c0 [11:4]
      spa->calib.c0 = getTwosComplement(((uint32_t)coef[0] << 4) | (((uint32_t)coef[1] >> 4) & 0x0F), 12);
      // 0x11 c1 [11:8] + 0x12 c1 [7:0]
      spa->calib.c1 = getTwosComplement((((uint32_t)coef[1] & 0x0F) << 8) | (uint32_t)coef[2], 12);

      // 0x13 c00 [19:12] + 0x14 c00 [11:4] + 0x15 c00 [3:0]
      spa->calib.c00 = getTwosComplement(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | (((uint32_t)coef[5] >> 4) & 0x0F), 20);

      // 0x15 c10 [19:16] + 0x16 c10 [15:8] + 0x17 c10 [7:0]
      spa->calib.c10 = getTwosComplement((((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | (uint32_t)coef[7], 20);
      spa->calib_idx++;
    break;
    case 1:
      // 0x18 c01 [15:8] + 0x19 c01 [7:0]
      spa->calib.c01 = getTwosComplement(((uint32_t)coef[0] << 8) | (uint32_t)coef[1], 16);

      // 0x1A c11 [15:8] + 0x1B c11 [7:0]
      spa->calib.c11 = getTwosComplement(((uint32_t)coef[2] << 8) | (uint32_t)coef[3], 16);

      // 0x1C c20 [15:8] + 0x1D c20 [7:0]
      spa->calib.c20 = getTwosComplement(((uint32_t)coef[4] << 8) | (uint32_t)coef[5], 16);

      // 0x1E c21 [15:8] + 0x1F c21 [7:0]
      spa->calib.c21 = getTwosComplement(((uint32_t)coef[6] << 8) | (uint32_t)coef[7], 16);
      spa->calib_idx++;
    break;
    case 2:
      // 0x20 c30 [15:8] + 0x21 c30 [7:0]
      spa->calib.c30 = getTwosComplement(((uint32_t)coef[0] << 8) | (uint32_t)coef[1], 16);
      if (spa->device == SPA06) {
      // 0x23 c31 [3:0] + 0x22 c31 [11:4]
      spa->calib.c31 = getTwosComplement(((uint32_t)coef[2] << 4) | (((uint32_t)coef[3] >> 4) & 0x0F), 12);

      // 0x23 c40 [11:8] + 0x24 c40 [7:0]
      spa->calib.c40 = getTwosComplement((((uint32_t)coef[3] & 0x0F) << 8) | (uint32_t)coef[4], 12);
      } else {
          spa->calib.c31 = 0;
          spa->calib.c40 = 0; 
      }
      spa->calib_idx++;
    break;
  } 
}


/**
 * @brief Compensate the raw pressure and temperature data 
 * return the compensated pressure data in integer data type.
 */
static void compensate_pressure(struct spa06_t *spa)
{
    // Calculate scaled measurement results.
  float Praw_sc = (float)spa->raw_pressure / raw_value_scale_factor(SPL06_PRESSURE_OVERSAMPLING);
  float Traw_sc = (float)spa->raw_temperature / raw_value_scale_factor(SPL06_TEMPERATURE_OVERSAMPLING);

  // See section 4.9.1, How to Calculate Compensated Pressure Values, of datasheet
  if (spa->device == SPA06) {
    spa->pressure = spa->calib.c00 + Praw_sc * (spa->calib.c10 + Praw_sc * (spa->calib.c20 + Praw_sc * (spa->calib.c30 + Praw_sc * spa->calib.c40))) + Traw_sc * spa->calib.c01 + Traw_sc * Praw_sc * (spa->calib.c11 + Praw_sc * (spa->calib.c21 + Praw_sc * spa->calib.c31));    
  } else {
    spa->pressure = spa->calib.c00 + Praw_sc * (spa->calib.c10 + Praw_sc * (spa->calib.c20 + Praw_sc * spa->calib.c30)) + Traw_sc * spa->calib.c01 + Traw_sc * Praw_sc * (spa->calib.c11 + Praw_sc * spa->calib.c21);
  }

  // See section 4.9.2, How to Calculate Compensated Temperature Values, of datasheet
  spa->temperature = spa->calib.c0 * 0.5f + spa->calib.c1 * Traw_sc;
}

/**
 * @brief Configure the spa06 device register by register
 * 
 * @param spa The spa06 instance
 * @return true When the configuration is completed
 * @return false Still busy configuring
 */
static bool spa06_config(struct spa06_t *spa) {
  // Only one transaction can be made per call to the periodic function 
  switch(spa->config_idx) {
    case 0:
      // PRS_CFG: pressure measurement rate (4 Hz) and oversampling 
      spa06_register_write(spa, SPL06_REG_PRESSURE_CFG, (SPL06_PRES_RATE_4HZ | SPL06_PRESSURE_OVERSAMPLING)); 
      spa->config_idx++;
      break;

    case 1: 
       // TMP_CFG: temperature measurement rate (32 Hz) and oversampling 
      spa06_register_write(spa, SPL06_REG_TEMPERATURE_CFG, (SPL06_PRES_RATE_4HZ | SPL06_TEMPERATURE_OVERSAMPLING));
      spa->config_idx++;
      break;

    case 2:
      uint8_t int_and_fifo_reg_value = 0;
      if (SPL06_TEMPERATURE_OVERSAMPLING > 3) { //SPL06_OVERSAMPLING_8X_T = 0x03 
          int_and_fifo_reg_value |= SPL06_TEMPERATURE_RESULT_BIT_SHIFT;
      }
      if (SPL06_PRESSURE_OVERSAMPLING > 3) { // SPL06_OVERSAMPLING_8X_P = 0x03
          int_and_fifo_reg_value |= SPL06_PRESSURE_RESULT_BIT_SHIFT;
      }
      spa06_register_write(spa, SPL06_REG_INT_AND_FIFO_CFG, int_and_fifo_reg_value);
      spa->config_idx++;
      break;

     case 3:
      spa06_register_write(spa, SPL06_REG_MODE_AND_STATUS, SPL06_MEAS_CON_PRE_TEM);
      spa->config_idx++;
      break;

    default:
      return true;
  }
  return false;
}

static bool spa06_get_calib(struct spa06_t *spa){
  // Only one transaction can be made per call to the periodic function 
  // Do the read of the coefficients in multiple parts, as the chip will return a read failure when trying to read all at once over I2C.
  switch(spa->calib_idx) {
    case 0:
      spa06_register_read(spa, SPL06_REG_CALIB_COEFFS_START, 8);
    break;
    case 1:
      spa06_register_read(spa, SPL06_REG_CALIB_COEFFS_START+8, 8);
    break;
    case 2:
      if(spa->device == SPA06){
        spa06_register_read(spa, SPL06_REG_CALIB_COEFFS_START+16, 5);
      }
      else if (spa->device == SPL06){
        spa06_register_read(spa, SPL06_REG_CALIB_COEFFS_START+16, 3);
      }
    break;
    default:
      return true;
  }
  return false;
}

int32_t raw_value_scale_factor(uint8_t oversampling)
{
    // From the datasheet page 13
    switch(oversampling)
    {
        case 0: return 524288;
        case 1: return 1572864;
        case 2: return 3670016;
        case 3: return 7864320;
        case 4: return 253952;
        case 5: return 516096;
        case 6: return 1040384;
        case 7: return 2088960;
        default: return -1; // invalid
    }
}


/**
 * @brief Write a register with a value
 * 
 * @param spa The spa06 instance
 * @param reg The register address
 * @param value The value to write to the register
 */
static void spa06_register_write(struct spa06_t *spa, uint8_t reg, uint8_t value) {

  spa->tx_buffer[1] = value;

  /* SPI transaction */
  if(spa->bus == SPA06_SPI) {
    spa->tx_buffer[0] = (reg & 0x7F); //write command (bit 7 = RW = '0')
    spa->spi.trans.output_length = 2;
    spa->spi.trans.input_length = 0;
    spi_submit(spa->spi.p, &(spa->spi.trans));
  }
  /* I2C transaction */
  else {
    spa->tx_buffer[0] = reg;
    i2c_transmit(spa->i2c.p, &(spa->i2c.trans), spa->i2c.slave_addr, 2);
  }
}

/**
 * @brief Read a register 
 * 
 * @param spa The spa06 instance
 * @param reg The register address
 * @param size The size to read 
 */
static void spa06_register_read(struct spa06_t *spa, uint8_t reg, uint16_t size) {

  /* SPI transaction */
  if(spa->bus == SPA06_SPI) {
    spa->tx_buffer[0] = reg | SPL06_READ_FLAG ; 
    spa->spi.trans.output_length = 2;
    spa->spi.trans.input_length = size+1; // already 1 is added for the transmission of the register to read
    spa->tx_buffer[1] = 0;
    spi_submit(spa->spi.p, &(spa->spi.trans));
  }
  /* I2C transaction */
  else {
    spa->tx_buffer[0] = reg ; 
    i2c_transceive(spa->i2c.p, &(spa->i2c.trans), spa->i2c.slave_addr, 1, size);
  }
}

static int32_t getTwosComplement(uint32_t raw, uint8_t length)
{
    if (raw & ((int)1 << (length - 1))) {
        return ((int32_t)raw) - ((int32_t)1 << length);
    }
    else {
        return raw;
    }
}