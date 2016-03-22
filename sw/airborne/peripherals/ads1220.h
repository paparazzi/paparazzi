/*
 * Copyright (C) 2014 Gautier Hattenberger, Alexandre Bustico
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
 *
 */

/**
 * @file peripherals/ads1220.h
 *
 * Driver for the ADS1220
 * 24-bits ADC from TI
 * SPI communication
 *
 */

#ifndef ADS1220_H
#define ADS1220_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/spi.h"

// Conf status
enum Ads1220ConfStatus {
  ADS1220_UNINIT = 0,
  ADS1220_SEND_RESET,
  ADS1220_INITIALIZING,
  ADS1220_INITIALIZED
};

// Input multiplexer
enum Ads1220Mux {
  ADS1220_MUX_AIN0_AIN1 = 0,
  ADS1220_MUX_AIN0_AIN2,
  ADS1220_MUX_AIN0_AIN3,
  ADS1220_MUX_AIN1_AIN2,
  ADS1220_MUX_AIN1_AIN3,
  ADS1220_MUX_AIN2_AIN3,
  ADS1220_MUX_AIN1_AIN0,
  ADS1220_MUX_AIN3_AIN2,
  ADS1220_MUX_AIN0_AVSS,
  ADS1220_MUX_AIN1_AVSS,
  ADS1220_MUX_AIN2_AVSS,
  ADS1220_MUX_AIN3_AVSS,
  ADS1220_MUX_REFP_REFN,
  ADS1220_MUX_AVDD_AVSS,
  ADS1220_MUX_AVDD_AVSS_2
};

// Gain config
enum Ads1220Gain {
  ADS1220_GAIN_1 = 0,
  ADS1220_GAIN_2,
  ADS1220_GAIN_4,
  ADS1220_GAIN_8,
  ADS1220_GAIN_16,
  ADS1220_GAIN_32,
  ADS1220_GAIN_64,
  ADS1220_GAIN_128
};

// Real gain table
static const float Ads1220GainTable[] = {
  1.0f, 2.0f, 4.0f, 8.0f, 16.0f, 32.0f, 64.0f, 128.0f
};

// Data sample in normal mode
enum Ads1220SampleRate {
  ADS1220_RATE_20_HZ = 0,
  ADS1220_RATE_45_HZ,
  ADS1220_RATE_90_HZ,
  ADS1220_RATE_175_HZ
};

// Conversion mode
enum Ads1220ConvMode {
  ADS1220_SINGLE_SHOT = 0,
  ADS1220_CONTINIOUS_CONVERSION
};

// Voltage reference
enum Ads1220VRef {
  ADS1220_VREF_INTERNAL = 0,
  ADS1220_VREF_EXTERNAL_REF,
  ADS1220_VREF_EXTERNAL_AIN,
  ADS1220_VREF_VDD
};

// IDAC current setting
enum Ads1220Idac {
  ADS1220_IDAC_OFF = 0,
  ADS1220_IDAC_10_uA,
  ADS1220_IDAC_50_uA,
  ADS1220_IDAC_100_uA,
  ADS1220_IDAC_250_uA,
  ADS1220_IDAC_500_uA,
  ADS1220_IDAC_1000_uA,
  ADS1220_IDAC_1500_uA
};

// IDAC routing
enum Ads1220Imux {
  ADS1220_IMUX_OFF = 0,
  ADS1220_IMUX_A0_RP1,
  ADS1220_IMUX_A1,
  ADS1220_IMUX_A2,
  ADS1220_IMUX_A3_RN1,
  ADS1220_IMUX_RP0,
  ADS1220_IMUX_RN0
};

/** ADS1220 configuration options
 */
struct Ads1220Config {
  enum Ads1220ConfStatus status;  ///< config status
  enum Ads1220Mux mux;            ///< input multiplexer
  enum Ads1220Gain gain;          ///< gain
  bool pga_bypass;              ///< bypass PGA (PGA enabled = 0)
  enum Ads1220SampleRate rate;    ///< data output rate
  enum Ads1220ConvMode conv;      ///< conversion mode
  enum Ads1220VRef vref;          ///< voltage ref
  enum Ads1220Idac idac;          ///< IDAC config
  enum Ads1220Imux i1mux;         ///< IDAC routing 1
  enum Ads1220Imux i2mux;         ///< IDAC routing 2
};


//enum Ads1220ClockFrequency {
//  CLOCK_328_KHZ = 0,
//  CLOCK_656_KHZ,
//  CLOCK_1312_KHZ
//};

#define ADS1220_BUFFER_LEN 5

struct Ads1220 {
  // SPI
  struct spi_periph *spi_p;                     ///< spi peripheral
  struct spi_transaction spi_trans;             ///< spi transaction
  volatile uint8_t tx_buf[ADS1220_BUFFER_LEN];  ///< transmit buffer
  volatile uint8_t rx_buf[ADS1220_BUFFER_LEN];  ///< receive buffer
  // Config
  struct Ads1220Config config;                  ///< configuration
  // Data
  uint32_t data;                                ///< raw ADC value
  volatile bool data_available;               ///< data ready flag
};

// Functions
extern void ads1220_init(struct Ads1220 *ads, struct spi_periph *spi_p, uint8_t slave_idx);
extern void ads1220_configure(struct Ads1220 *ads);
extern void ads1220_read(struct Ads1220 *ads);
extern void ads1220_event(struct Ads1220 *ads);

/// convenience function: read or start configuration if not already initialized
static inline void ads1220_periodic(struct Ads1220 *ads)
{
  if (ads->config.status == ADS1220_INITIALIZED) {
    ads1220_read(ads);
  } else {
    ads1220_configure(ads);
  }
}

#endif // ADS1220_H

