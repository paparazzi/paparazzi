/*
 * Chris Efstathiou hendrixgr@gmail.com
 * Florian Sansou florian.sansou@enac.fr
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
 * @file peripherals/bmp280.c
 * @brief Sensor driver for BMP280 sensor 
 *
 *
 */

#include "peripherals/bmp280.h"

/** local function to extract raw data from i2c buffer
 *  and compute compensation with selected precision
 */
static void parse_sensor_data(struct bmp280_t *bmp, uint8_t *data);
static void parse_calib_data(struct bmp280_t *bmp, uint8_t *data);
PRINT_CONFIG_MSG("BMP280 uses double precision compensation")
static double compensate_pressure(struct bmp280_t *bmp);
static double compensate_temperature(struct bmp280_t *bmp);
static bool bmp280_config(struct bmp280_t *bmp);
static void bmp280_register_write(struct bmp280_t *bmp, uint8_t reg, uint8_t value);
static void bmp280_register_read(struct bmp280_t *bmp, uint8_t reg, uint16_t size);


/**
 * @brief Initialize the bmp280 sensor instance
 * 
 * @param bmp The structure containing the configuration of the bmp280 instance
 */
void bmp280_init(struct bmp280_t *bmp)
{

    bmp->data_available = false;
    bmp->initialized = false;
    bmp->status = BMP280_STATUS_UNINIT;

    bmp->config_idx = 0;

  /* SPI setup */
  if(bmp->bus == BMP280_SPI) {
    bmp->spi.trans.cpol = SPICpolIdleHigh;
    bmp->spi.trans.cpha = SPICphaEdge2;
    bmp->spi.trans.dss = SPIDss8bit;
    bmp->spi.trans.bitorder = SPIMSBFirst;
    bmp->spi.trans.cdiv = SPIDiv16;

    bmp->spi.trans.select = SPISelectUnselect;
    bmp->spi.trans.slave_idx = bmp->spi.slave_idx;
    bmp->spi.trans.output_length = 0;
    bmp->spi.trans.input_length = 0;
    bmp->spi.trans.before_cb = NULL;
    bmp->spi.trans.after_cb = NULL;
    bmp->spi.trans.input_buf = bmp->spi.rx_buf;
    bmp->spi.trans.output_buf = bmp->spi.tx_buf;
    bmp->spi.trans.status = SPITransDone;

    // in SPI read, the first byte is garbage because writing the register address
    bmp->rx_buffer = &bmp->spi.rx_buf[1];
    bmp->tx_buffer = bmp->spi.tx_buf;
    bmp->rx_length = &bmp->spi.trans.input_length;
  }
  /* I2C setup */
  else {
    
    bmp->i2c.trans.slave_addr = bmp->i2c.slave_addr; // slave address
    bmp->i2c.trans.status = I2CTransDone;  // set initial status: Done

    bmp->rx_buffer = (uint8_t *)bmp->i2c.trans.buf;
    bmp->tx_buffer = (uint8_t *)bmp->i2c.trans.buf;
    bmp->rx_length = &bmp->i2c.trans.len_r;

  }
}

/**
 * @brief Should be called periodically to request sensor readings
 * - First detects the sensor using WHO_AM_I reading
 * - Configures the sensor according the users requested configuration
 * - Requests a sensor reading 
 * 
 * @param bmp The bmp280 instance
 */
void bmp280_periodic(struct bmp280_t *bmp)
{

  /* Idle */
  if((bmp->bus == BMP280_SPI && bmp->spi.trans.status == SPITransDone) || 
     (bmp->bus == BMP280_I2C && bmp->i2c.trans.status == I2CTransDone)) {

      switch (bmp->status) {
        case BMP280_STATUS_UNINIT:
          bmp->data_available = false;
          bmp->initialized = false;
          bmp->status = BMP280_STATUS_IDLE;
          break;
        
        case BMP280_STATUS_IDLE:
          /* Request WHO_AM_I */
          bmp280_register_read(bmp, BMP280_CHIP_ID_REG_ADDR, 1);
          break;

        case BMP280_STATUS_GET_CALIB:
          // request calibration data
          bmp280_register_read(bmp, BMP280_CALIB_LSB_DATA_ADDR, BMP280_CALIB_DATA_LEN);
          //process in bmp280_event()
          break;

        case BMP280_STATUS_CONFIGURE:
          /* Start configuring */
          if(bmp280_config(bmp)) {
            bmp->status = BMP280_STATUS_READ_STATUS_REG;
            bmp->initialized = true;
          }
          break;

        case  BMP280_STATUS_READ_STATUS_REG:
          // READ THE STATUS BYTE
          bmp280_register_read(bmp, BMP280_STATUS_REG_ADDR, 1);
          break;

        case  BMP280_STATUS_READ_DATA_REGS:
          // READ ALL 6 DATA REGISTERS
          bmp280_register_read(bmp, BMP280_DATA_START_REG_ADDR, BMP280_P_T_DATA_LEN);
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
 * @param bmp The bmp280 instance
 */
void bmp280_event(struct bmp280_t *bmp)
{
  /* Successful transfer */
  if((bmp->bus == BMP280_SPI && bmp->spi.trans.status == SPITransSuccess) || 
     (bmp->bus == BMP280_I2C && bmp->i2c.trans.status == I2CTransSuccess)) {
      switch (bmp->status) {

        case BMP280_STATUS_IDLE:
          /* WHO_AM_I */
          if(bmp->rx_buffer[0] == BMP280_ID_NB) {
            bmp->device = BMP_280;
            bmp->status = BMP280_STATUS_GET_CALIB;
          } else {
            bmp->status = BMP280_STATUS_IDLE;
          }
          break;

        case BMP280_STATUS_GET_CALIB:
          // compute calib
          parse_calib_data(bmp, &bmp->rx_buffer[0]); 
          bmp->status = BMP280_STATUS_CONFIGURE;
          break;

        case BMP280_STATUS_CONFIGURE:
          if(bmp280_config(bmp)) {
            bmp->status = BMP280_STATUS_READ_STATUS_REG;
            bmp->initialized = true;
          }
          break;

        case BMP280_STATUS_READ_STATUS_REG:
          // check status byte
          if ((bmp->rx_buffer[0] & (BMP280_EOC_BIT | BMP280_NVRAM_COPY_BIT)) == 0) {
            bmp->status = BMP280_STATUS_READ_DATA_REGS;
          }
          break;

        case BMP280_STATUS_READ_DATA_REGS:
          // parse sensor data, compensate temperature first, then pressure
          parse_sensor_data(bmp, &bmp->rx_buffer[0]);
          compensate_temperature(bmp);
          compensate_pressure(bmp);
          bmp->data_available = true;
          bmp->status = BMP280_STATUS_READ_STATUS_REG;
          break;

        default:
          bmp->status = BMP280_STATUS_GET_CALIB; // just to avoid the compiler's warning message
          break;
      }
      if(bmp->bus == BMP280_I2C){
        bmp->i2c.trans.status = I2CTransDone; 
      }
      else{
        bmp->spi.trans.status = SPITransDone;
      }
      
    } else if ((bmp->bus == BMP280_SPI && bmp->spi.trans.status == SPITransFailed) || 
               (bmp->bus == BMP280_I2C && bmp->i2c.trans.status == I2CTransFailed)) {
      /* try again */
      if (!bmp->initialized) {
        bmp->status = BMP280_STATUS_UNINIT;
      }
      if(bmp->bus == BMP280_I2C){
        bmp->i2c.trans.status = I2CTransDone; 
      }
      else{
        bmp->spi.trans.status = SPITransDone;
      }
    }

  return;
}

static void parse_sensor_data(struct bmp280_t *bmp, uint8_t *data)
{
  /* Temporary variables to store the sensor data */
  uint32_t data_xlsb;
  uint32_t data_lsb;
  uint32_t data_msb;

  // BMP280 HAS THE 6 DATA REGISTERS START AT F7 AND GOING UP TO FC MSB FIRST THEN LSB AND LAST THE XLSB BYTE.
  // THE FIRST THREE BYTES ARE THE PRESSURE AND THE NEXT 3 THE TEMPERATURE.
  /* Store the parsed register values for pressure data */
  data_msb = (uint32_t)data[0] << 16;
  data_lsb = (uint32_t)data[1] << 8;
  data_xlsb = (uint32_t)data[2];
  bmp->raw_pressure = (int32_t)((data_msb | data_lsb | data_xlsb) >> 4);

  /* Store the parsed register values for temperature data */
  data_msb = (uint32_t)data[3] << 16;
  data_lsb = (uint32_t)data[4] << 8;
  data_xlsb = (uint32_t)data[5];
  bmp->raw_temperature = (int32_t)((data_msb | data_lsb | data_xlsb) >> 4);
}


/**
 *  @brief This internal API is used to parse the calibration data, compensates
 *  it and store it in device structure (float version)
 */
static void parse_calib_data(struct bmp280_t *bmp, uint8_t *data)
{
  bmp->calib.dig_t1 = BMP280_CONCAT_BYTES(data[1], data[0]);
  bmp->calib.dig_t2 = (int16_t)BMP280_CONCAT_BYTES(data[3], data[2]);
  bmp->calib.dig_t3 = (int16_t)BMP280_CONCAT_BYTES(data[5], data[4]);

  bmp->calib.dig_p1 = BMP280_CONCAT_BYTES(data[7], data[6]);
  bmp->calib.dig_p2 = (int16_t)BMP280_CONCAT_BYTES(data[9], data[8]);
  bmp->calib.dig_p3 = (int16_t)BMP280_CONCAT_BYTES(data[11], data[10]);
  bmp->calib.dig_p4 = (int16_t)BMP280_CONCAT_BYTES(data[13], data[12]);
  bmp->calib.dig_p5 = (int16_t)BMP280_CONCAT_BYTES(data[15], data[14]);
  bmp->calib.dig_p6 = (int16_t)BMP280_CONCAT_BYTES(data[17], data[16]);
  bmp->calib.dig_p7 = (int16_t)BMP280_CONCAT_BYTES(data[19], data[18]);
  bmp->calib.dig_p8 = (int16_t)BMP280_CONCAT_BYTES(data[21], data[20]);
  bmp->calib.dig_p9 = (int16_t)BMP280_CONCAT_BYTES(data[23], data[22]);

  return;
}

/**
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in float data type.
 */
static double compensate_temperature(struct bmp280_t *bmp)
{
  double var1 = (((double)bmp->raw_temperature / 16384.0) - ((double)bmp->calib.dig_t1 / 1024.0)) * ((double)bmp->calib.dig_t2);
  double var2 = ((double)bmp->raw_temperature / 131072.0) - ((double)bmp->calib.dig_t1 / 8192.0);
  var2 = (var2 * var2) * (double)bmp->calib.dig_t3;
  int64_t t_fine = (int64_t)(var1 + var2);

  /* Store t_lin in dev. structure for pressure calculation */
  bmp->calib.t_fine = t_fine;
  /* Store compensated temperature in float in structure */
  bmp->temperature = (((var1 + var2) / 5120.f) * 100);

  return (double)bmp->temperature;
}

/**
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 */
static double compensate_pressure(struct bmp280_t *bmp)
{
  double var1;
  double var2;
  double p;

  var1 = ((double)bmp->calib.t_fine / 2) - 64000.0;
  var2 = (var1 * var1 * (double)bmp->calib.dig_p5) / 32768.0;
  var2 = var2 + (var1 * (double)bmp->calib.dig_p5 * 2.0);
  var2 = (var2 / 4.0) + ((double)bmp->calib.dig_p4 * 65536.0);
  var1 = (((double)bmp->calib.dig_p3 * var1 * (var1 / 524288.0)) + ((double)bmp->calib.dig_p2 * var1)) / 524288.0;
  var1 = (1 + (var1 / 32768.0)) * (double)bmp->calib.dig_p1;
  p = 1048576.0 - (double)bmp->raw_pressure;
  p = (p - (var2 / 4096.0)) * (6250.0 / var1);
  var1 = ((double)bmp->calib.dig_p9 * p) * (p / 2147483648.0);
  var2 = (p * ((double)bmp->calib.dig_p8)) / 32768.0;
  p = p + ((var1 + var2 + (double)bmp->calib.dig_p7) / 16.0);
  bmp->pressure = p;

  return (p);
}

/**
 * @brief Configure the BMP280 device register by register
 * 
 * @param bmp The bmp280 instance
 * @return true When the configuration is completed
 * @return false Still busy configuring
 */
static bool bmp280_config(struct bmp280_t *bmp) {
  // Only one transaction can be made per call to the periodic function 
  switch(bmp->config_idx) {
    case 0:
      // From datasheet, recommended config for drone usecase:
      // osrs_p = 16, osrs_t = 2
      bmp280_register_write(bmp, BMP280_CTRL_MEAS_REG_ADDR, (BMP280_OVERSAMPLING_2X_T | BMP280_OVERSAMPLING_16X_P | BMP280_POWER_NORMAL_MODE)); 
      bmp->config_idx++;
      break;

    case 1: 
      // IIR filter = 16 
      bmp280_register_write(bmp, BMP280_CONFIG_REG_ADDR, (BMP280_INACTIVITY_HALF_MS | BMP280_IIR_FILTER_COEFF_16));
      bmp->config_idx++;
      break;

    default:
      return true;
  }
  return false;
}


/**
 * @brief Write a register with a value
 * 
 * @param bmp The bmp280 instance
 * @param reg The register address
 * @param value The value to write to the register
 */
static void bmp280_register_write(struct bmp280_t *bmp, uint8_t reg, uint8_t value) {

  bmp->tx_buffer[1] = value;
  
  /* SPI transaction */
  if(bmp->bus == BMP280_SPI) {
    bmp->tx_buffer[0] = (reg & 0x7F); //write command (bit 7 = RW = '0')
    bmp->spi.trans.output_length = 2;
    bmp->spi.trans.input_length = 0;
    spi_submit(bmp->spi.p, &(bmp->spi.trans));
  }
  /* I2C transaction */
  else {
     bmp->tx_buffer[0] = reg;
    i2c_transmit(bmp->i2c.p, &(bmp->i2c.trans), bmp->i2c.slave_addr, 2);
  }
}

/**
 * @brief Read a register 
 * 
 * @param bmp The bmp280 instance
 * @param reg The register address
 * @param size The size to read (already 1 is added for the transmission of the register to read)
 */
static void bmp280_register_read(struct bmp280_t *bmp, uint8_t reg, uint16_t size) {

  bmp->tx_buffer[0] = reg | BMP280_READ_FLAG;
  /* SPI transaction */
  if(bmp->bus ==   BMP280_SPI) {
    bmp->spi.trans.output_length = 2;
    bmp->spi.trans.input_length = size+1;
    bmp->tx_buffer[1] = 0;
    spi_submit(bmp->spi.p, &(bmp->spi.trans));
  }
  /* I2C transaction */
  else {
    i2c_transceive(bmp->i2c.p, &(bmp->i2c.trans), bmp->i2c.slave_addr, 1, size);
  }
}

