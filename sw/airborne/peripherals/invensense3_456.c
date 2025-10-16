/*
 * Copyright (C) 2022 Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file peripherals/invensense3_456.c
 *
 * Driver for the Invensense 456XY IMUs:
 * - ICM45686
 */

#include "peripherals/invensense3_456.h"
#include "peripherals/invensense3_456_regs.h"
#include "math/pprz_isa.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/gpio_arch.h"
#include "std.h"


#include <string.h>
#include "modules/datalink/telemetry.h"

/* Local functions */
static void invensense3_456_parse_fifo_data(struct invensense3_456_t *inv, uint8_t *data, uint16_t samples);
static void invensense3_456_set_scalings(struct invensense3_456_t *inv);
static bool invensense3_456_register_write(struct invensense3_456_t *inv, uint8_t reg, uint8_t value);
static bool invensense3_456_register_read(struct invensense3_456_t *inv, uint8_t reg, uint16_t size);
static bool invensense3_456_config(struct invensense3_456_t *inv);
static int samples_from_odr(int odr);


/**
 * @brief Initialize the invensense v3 sensor instance
 * 
 * @param inv The structure containing the configuration of the invensense v3 instance
 */
void invensense3_456_init(struct invensense3_456_t *inv) {
  /* General setup */
  inv->status = INVENSENSE3_456_IDLE;
  inv->device = INVENSENSE3_456_UNKOWN;
  inv->config_idx = 0;
  
  /* SPI setup */
  inv->spi.trans.cpol = SPICpolIdleHigh;
  inv->spi.trans.cpha = SPICphaEdge2;
  inv->spi.trans.dss = SPIDss8bit;
  inv->spi.trans.bitorder = SPIMSBFirst;
  inv->spi.trans.cdiv = SPIDiv16;

  inv->spi.trans.select = SPISelectUnselect;
  inv->spi.trans.slave_idx = inv->spi.slave_idx;
  inv->spi.trans.output_length = 0;
  inv->spi.trans.input_length = 0;
  inv->spi.trans.before_cb = NULL;
  inv->spi.trans.after_cb = NULL;
  inv->spi.trans.input_buf = inv->spi.rx_buf;
  inv->spi.trans.output_buf = inv->spi.tx_buf;
  inv->spi.trans.status = SPITransDone;

  inv->rx_buffer = inv->spi.rx_buf;
  inv->tx_buffer = inv->spi.tx_buf;
  inv->rx_length = &inv->spi.trans.input_length;

  inv->sample_numbers = samples_from_odr((int)inv->imu_odr);
}

/**
 * @brief Should be called periodically to request sensor readings
 * - First detects the sensor using WHO_AM_I reading
 * - Configures the sensor according the users requested configuration
 * - Requests a sensor reading by reading the FIFO_COUNT register
 * 
 * @param inv The invensense v3 instance
 */
void invensense3_456_periodic(struct invensense3_456_t *inv) {
  /* Idle */
  if(inv->spi.trans.status == SPITransDone) {
    /* Depending on the status choose what to do */
    switch(inv->status) {
      case INVENSENSE3_456_IDLE:
        /* Request WHO_AM_I */
        invensense3_456_register_read(inv, INV3REG_456_WHOAMI, 1);
        break;

      case INVENSENSE3_456_CONFIG:
        /* Start configuring */
        if(invensense3_456_config(inv)) {
          inv->status = INVENSENSE3_456_RUNNING;
        }
        break;

      case INVENSENSE3_456_RUNNING:
        {
          static const uint16_t max_bytes = sizeof(inv->spi.rx_buf) - 3;
          int read_bytes = (inv->sample_numbers + inv->timer) * sizeof(struct FIFODataHighRes);
          read_bytes = Min(max_bytes, read_bytes);
          // round to the packet boundaries
          read_bytes -= read_bytes % sizeof(struct FIFODataHighRes);
          invensense3_456_register_read(inv, INV3REG_456_FIFO_COUNTH, read_bytes + 2);
          inv->timer = 0; // reset leftover bytes
          break;
        }
      
      default: break;
    }
  }
}

/**
 * @brief Should be called in the event thread
 * - Checks the response of the WHO_AM_I reading
 * - Configures the sensor and reads the responses
 * - Parse and request the sensor data from the FIFO buffers
 * 
 * @param inv The invensense v3 instance
 */
void invensense3_456_event(struct invensense3_456_t *inv) {

  /* Successful transfer */
  if (inv->spi.trans.status == SPITransSuccess) {
    /* Set the transaction as done */
    inv->spi.trans.status = SPITransDone;
    
    /* Look at the results */
    switch(inv->status) {
      case INVENSENSE3_456_IDLE:
        /* Check the response of the WHO_AM_I */
        inv->status = INVENSENSE3_456_CONFIG;
        if(inv->rx_buffer[1] == INV3_456_WHOAMI_ICM45686) {
          inv->device = INVENSENSE3_456_ICM45686;
        } else {
          inv->status = INVENSENSE3_456_IDLE;
        }

        /* Set the scaling */
        if(inv->status == INVENSENSE3_456_CONFIG)
          invensense3_456_set_scalings(inv);
        break;

      case INVENSENSE3_456_CONFIG:
        /* Apply the next configuration register */
        if(invensense3_456_config(inv)) {
          inv->status = INVENSENSE3_456_RUNNING;
        }
        break;

      case INVENSENSE3_456_RUNNING:
        {
          /* Parse the results */
          uint16_t n_samples = (uint16_t)inv->rx_buffer[1] | inv->rx_buffer[2] << 8;
          uint16_t fifo_bytes = n_samples * sizeof(struct FIFODataHighRes);
          
          inv->timer = n_samples;   // samples left in FIFO

          // Not enough data in FIFO
          if(n_samples == 0) {
            return;
          }

          // We read an incorrect length (try again)
          if(fifo_bytes > 4096) {
            invensense3_456_register_read(inv, INV3REG_456_FIFO_COUNTH, 2);
            return;
          }

          // Parse the data
          if(*inv->rx_length > 3) {
            uint16_t nb_packets_read = (*inv->rx_length - 3) / sizeof(struct FIFODataHighRes);
            invensense3_456_parse_fifo_data(inv, &inv->rx_buffer[3], Min(n_samples, nb_packets_read));
            inv->timer -= nb_packets_read;
          }
        }
        break;
      default:
        inv->status = INVENSENSE3_456_IDLE;
        break;
    }
  }
  
  /* Failed transaction */
  if (inv->spi.trans.status == SPITransFailed) {

    /* Set the transaction as done */
    inv->spi.trans.status = SPITransDone;

    /* Retry or ignore */
    switch(inv->status) {
      case INVENSENSE3_456_CONFIG:
        /* Try again */
        if(invensense3_456_config(inv)) {
          inv->status = INVENSENSE3_456_RUNNING;
        }
        break;
      case INVENSENSE3_456_IDLE:
      case INVENSENSE3_456_RUNNING: 
        /* Ignore while idle/running */
      default:
        break;
    }
  }
}

// Helper for reassembling the 20-bit data retrieved from the FIFO buffer
// REF: https://github.com/ArduPilot/ardupilot/blob/85a8a55611a95d4c59052f79f56744a93a9c5a63/libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp#L489
static inline int32_t reassemble_uint20(uint8_t msb, uint8_t bits, uint8_t lsb) {
    uint32_t value20bit = ((uint32_t)msb << 12U) | ((uint32_t)bits << 4U) | (uint32_t)lsb;
    int32_t value32bit;
    // Check the sign bit (MSB)
    if (value20bit & 0x80000) { // MSB is set (negative value)
        // Extend the sign bit to the upper 12 bits of the 32-bit integer
        value32bit = (int32_t)(value20bit | 0xFFF00000);
    } else { // MSB is not set (positive value)
        // Zero-fill the upper 12 bits of the 32-bit integer
        value32bit = value20bit;
    }

    return value32bit;
}

/**
 * @brief Parse the FIFO buffer data
 * 
 * @param inv The invensense v3 instance
 * @param data The FIFO buffer data to parse
 * @param len The length of the FIFO buffer
 */
static void invensense3_456_parse_fifo_data(struct invensense3_456_t *inv, uint8_t *data, uint16_t samples) {
  static struct Int32Vect3 accel[INVENSENSE3_456_FIFO_BUFFER_LEN] = {0};
  static struct Int32Rates gyro[INVENSENSE3_456_FIFO_BUFFER_LEN] = {0};

  samples = Min(samples, INVENSENSE3_456_FIFO_BUFFER_LEN);

  // Go through the different samples
  int32_t temp = 0;
  int sample_idx = 0;
  for(uint8_t sample = 0; sample < samples; sample++) { 
    struct FIFODataHighRes *d = (struct FIFODataHighRes *)data;

    if ((d->header & 0xFC) != 0x78) { // ACCEL_EN | GYRO_EN | HIRES_EN | TMST_FIELD_EN
        // no or bad data
        return;
    } else {
        // Valid FIFO packet - re-assemble the data 
        gyro[sample_idx].p = reassemble_uint20(d->gyro[1], d->gyro[0], d->gx);
        gyro[sample_idx].q = reassemble_uint20(d->gyro[3], d->gyro[2], d->gy);
        gyro[sample_idx].r = reassemble_uint20(d->gyro[5], d->gyro[4], d->gz);
        
        accel[sample_idx].x = reassemble_uint20(d->accel[1], d->accel[0], d->ax);
        accel[sample_idx].y = reassemble_uint20(d->accel[3], d->accel[2], d->ay);
        accel[sample_idx].z = reassemble_uint20(d->accel[5], d->accel[4], d->az);

        // Compute the average temperature over the sampling period by summing and then dividing over the number of valid samples
        temp += d->temperature;

        sample_idx++;
      }

      data += sizeof(struct FIFODataHighRes);
  }

  // ADC temp * temp sensitivity + temp zero
  // From the datasheet for high-res 16-bit: "Temperature in Degrees Centigrade = (TEMP_DATA / 128) + 25"
  float temp_f = ((float)temp / sample_idx) / 128.f + 25.f; 

  // Send the obtained samples over ABI
  uint32_t now_ts = get_sys_time_usec();
  AbiSendMsgIMU_GYRO_RAW(inv->abi_id, now_ts, gyro, sample_idx, inv->imu_samplerate, temp_f);
  AbiSendMsgIMU_ACCEL_RAW(inv->abi_id, now_ts, accel, sample_idx, inv->imu_samplerate, temp_f);
  AbiSendMsgTEMPERATURE(inv->abi_id, temp_f);
}

/**
 * @brief This sets the correct scalings
 * 
 * @param inv The invensense v3 instance
 */
static void invensense3_456_set_scalings(struct invensense3_456_t *inv) {
  switch (inv->imu_odr) {
    case INVENSENSE3_456_ODR_6_4KHZ:
      inv->imu_samplerate = 6400;
      break; 
    case INVENSENSE3_456_ODR_3_2KHZ:
      inv->imu_samplerate = 3200;
      break;
    case INVENSENSE3_456_ODR_1_6KHZ:
      inv->imu_samplerate = 1600;
      break;
    case INVENSENSE3_456_ODR_800HZ:
      inv->imu_samplerate = 800;
      break;
    case INVENSENSE3_456_ODR_400HZ:
      inv->imu_samplerate = 400;
      break;
    case INVENSENSE3_456_ODR_200HZ:
      inv->imu_samplerate = 200;
      break;
    case INVENSENSE3_456_ODR_100HZ:
      inv->imu_samplerate = 100;
      break;
    case INVENSENSE3_456_ODR_50HZ:
      inv->imu_samplerate = 50;
      break;
    case INVENSENSE3_456_ODR_25HZ:
      inv->imu_samplerate = 25;
      break;
    default:
      /* Unsupported ODR! */
      inv->imu_samplerate = 0;
      break;

  }
  
  /* In the 20-bit high-resolution mode, the scaling is always set to 4000dps/32G no matter what we set up, hence we don't expose any config for this */
  /* 20 bits of data, and one for sign -> 32G*9.81(m/s^2)/g / 2^(20-1) = m/s^2 PER LSB, with the appropriate scaling for our internal representatations (e.g. 10ths of m/s for accel as of writing) */
  static const struct FloatVect3 accel_scale_hr = {ACCEL_BFP_OF_REAL(32*9.81/(1 << 19)), ACCEL_BFP_OF_REAL(32*9.81/(1 << 19)), ACCEL_BFP_OF_REAL(32*9.81/(1 << 19))};                     // 32G high-res (20 bit mode)
  static const struct FloatRates gyro_scale_hr = {RATE_BFP_OF_REAL(RadOfDeg(4000)/(1 << 19)), RATE_BFP_OF_REAL(RadOfDeg(4000)/(1 << 19)), RATE_BFP_OF_REAL(RadOfDeg(4000)/(1 << 19))};                     // 4000dps high-res (20 bit mode)
  imu_set_defaults_accel(inv->abi_id, NULL, NULL, &accel_scale_hr);
  imu_set_defaults_gyro(inv->abi_id, NULL, NULL, &gyro_scale_hr);
}

/**
 * @brief Write a register
 * 
 * @param inv The invensense v3 instance
 * @param reg The address of the register to write to
 * @param value The value to write to the register
 * @return true Whenever the register write was started
 */
static bool invensense3_456_register_write(struct invensense3_456_t *inv, uint8_t reg, uint8_t value) {

  inv->tx_buffer[0] = reg;
  inv->tx_buffer[1] = value;
  
  /* SPI transaction */
  inv->spi.trans.output_length = 2;
  inv->spi.trans.input_length = 0;
  spi_submit(inv->spi.p, &(inv->spi.trans));

  return true;
}

/**
 * @brief Write to a register in a specific bank
 * 
 * @param inv The invensense v3 instance
 * @param bank_addr The address of the bank to write to
 * @param reg The address of the register to write to
 * @param value The value to write to the register
 * @return true Whenever the register write was started
 */
static bool invensense3_register_write_bank(struct invensense3_456_t *inv, uint16_t bank_addr, uint16_t reg, uint8_t val) {
  // combine addr
    uint16_t addr = bank_addr + reg;
    inv->tx_buffer[0] = INV3REG_456_IREG_ADDRH; // indirect register address high
    inv->tx_buffer[1] = (uint8_t)(addr >> 8);
    inv->tx_buffer[2] = (uint8_t)(addr & 0xFF);
    inv->tx_buffer[3] = val;

    // set indirect register address
    inv->spi.trans.output_length = 4;
    inv->spi.trans.input_length = 0;
    spi_submit(inv->spi.p, &(inv->spi.trans));

    return true;
}

/**
 * @brief Read a register based on a combined bank/regsiter value
 * 
 * @param inv The invensense v3 instance
 * @param reg The address of the register to read
 * @param size The size to read (already 1 is added for the transmission of the register to read)
 * @return true If we initiated the register read succesfully
 * @return false First we are busy switching the register bank
 */
static bool invensense3_456_register_read(struct invensense3_456_t *inv, uint8_t reg, uint16_t size) {
  inv->tx_buffer[0] = reg | INV3_456_READ_FLAG;
  /* SPI transaction */
  inv->spi.trans.output_length = 2;
  inv->spi.trans.input_length = 1 + size;
  inv->tx_buffer[1] = 0;
  spi_submit(inv->spi.p, &(inv->spi.trans));

  return true;
}

/**
 * @brief Configure the Invensense 3 device register by register
 * 
 * REF: https://github.com/ArduPilot/ardupilot/blob/85a8a55611a95d4c59052f79f56744a93a9c5a63/libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp#L933
 * @param inv The invensense v3 instance
 * @return true When the configuration is completed
 * @return false Still busy configuring
 */
static bool invensense3_456_config(struct invensense3_456_t *inv) {
  switch(inv->config_idx) {
    case 0:
      /* Reset the device */
      if(invensense3_456_register_write(inv, INV3REG_456_REG_MISC2, 0x02))
        inv->config_idx++;
      inv->timer = get_sys_time_usec();
      break;

    case 1: 
      /* Because reset takes time wait ~5ms */
      if((get_sys_time_usec() - inv->timer) < 5e3)
        break;

      /* Start the accel en gyro in low noise mode */
      if(invensense3_456_register_write(inv, INV3REG_456_PWR_MGMT0, 0x0f))
        inv->config_idx++;
      inv->timer = get_sys_time_usec();
      break;
    case 2:
      /* Because starting takes time wait ~1ms */
      if((get_sys_time_usec() - inv->timer) < 1e3)
        break;

      // Disable FIFO first
      if (invensense3_456_register_write(inv, INV3REG_456_FIFO_CONFIG3, 0x00))
        inv->config_idx++;
      break;
    case 3:
      if (invensense3_456_register_write(inv, INV3REG_456_FIFO_CONFIG0, 0x00))
        inv->config_idx++;
      break;
    case 4: 
      /* Configure the gyro ODR/FS */
      if(invensense3_456_register_write(inv, INV3REG_456_GYRO_CONFIG0, (0x00 << GYRO_FS_SEL_SHIFT) | (inv->imu_odr << GYRO_ODR_SHIFT)))
        inv->config_idx++;
      break;
    case 5: {
      /* Configure the accel ODR/FS */
      if (invensense3_456_register_write(inv, INV3REG_456_ACCEL_CONFIG0, (0x00 << ACCEL_FS_SEL_SHIFT) |  (inv->imu_odr << ACCEL_ODR_SHIFT)))
        inv->config_idx++;
      break;
    }
    case 6:
      /* SMC_CONTROL_0 -> TMST_EN needs to be set to 1, default value is 0x60 which does not set this bit to 1; ardupilot reads whole register, we just assume it's at the default */
      if (invensense3_register_write_bank(inv, INV3BANK_456_IPREG_TOP1_ADDR, 0x58, 0x60 | 0x01))
        inv->config_idx++;
      break;
    case 7:
      /* FIFO content: enable accel, gyro, temperature + HI-RES mode */
      if(invensense3_456_register_write(inv, INV3REG_456_FIFO_CONFIG3 , INV_456_BASE_FIFO3_CONFIG_VALUE))
        inv->config_idx++;
      break;
    case 8:
      // FIFO enabled - stop-on-full, disable bypass and 2K FIFO
      if (invensense3_456_register_write(inv, INV3REG_456_FIFO_CONFIG0, (2 << 6) | 0x07))
        inv->config_idx++;
      break;
    case 9:
      /* Enable interplator & AAF on gyro */
      if (invensense3_register_write_bank(inv, INV3BANK_456_IPREG_SYS1_ADDR, 0xA6, (0x1B & ~(0x3 << 5)) | (0x2 << 5)))
        inv->config_idx++;
      break;
    case 10:
      /* Enable interplator & AAF on accel */
      if (invensense3_register_write_bank(inv, INV3BANK_456_IPREG_SYS2_ADDR, 0x7B, (0x14 & ~0x3) | 0x2))
        inv->config_idx++;
      break;
    case 11:
      // Enable FIFO sensor registers
      if (invensense3_456_register_write(inv, INV3REG_456_FIFO_CONFIG3, INV_456_BASE_FIFO3_CONFIG_VALUE | INV_456_FIFO_IF_EN))
        inv->config_idx++;
      break;
    default:
      inv->timer = 0;
      return true;
  }
  return false;
}

static int samples_from_odr(int odr) {
  switch (odr) {
    case INVENSENSE3_456_ODR_6_4KHZ:
      return ceilf(6400 / PERIODIC_FREQUENCY);
    case INVENSENSE3_456_ODR_3_2KHZ:
      return ceilf(3200 / PERIODIC_FREQUENCY);
    case INVENSENSE3_456_ODR_1_6KHZ:
      return ceilf(1600 / PERIODIC_FREQUENCY);
    case INVENSENSE3_456_ODR_800HZ:
      return ceilf(800 / PERIODIC_FREQUENCY);
    case INVENSENSE3_456_ODR_400HZ:
      return ceilf(400 / PERIODIC_FREQUENCY);
    case INVENSENSE3_456_ODR_200HZ:
      return ceilf(200 / PERIODIC_FREQUENCY);
    case INVENSENSE3_456_ODR_100HZ:
      return ceilf(100 / PERIODIC_FREQUENCY);
    case INVENSENSE3_456_ODR_50HZ:
      return ceilf(50 / PERIODIC_FREQUENCY);
    case INVENSENSE3_456_ODR_25HZ:
      return ceilf(25 / PERIODIC_FREQUENCY);
    default:
      // Error
      return 0;
  }
}