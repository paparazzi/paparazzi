/*
 * Copyright (C) 2019 Tom van Dijk <tomvand@users.noreply.github.com>
 *
 * This code is based on the betaflight cc2500 and FrskyX implementation.
 * https://github.com/betaflight/betaflight
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

#ifndef RADIO_CONTROL_CC2500_SETTINGS_H
#define RADIO_CONTROL_CC2500_SETTINGS_H

#include "cc2500_compat.h"

#include <stdint.h>

/* Paparazzi settings */
void cc2500_settings_init(void);

struct cc2500_settings_persistent_s {
  uint32_t bindVars;
  uint32_t bindHopData[13];
};
extern struct cc2500_settings_persistent_s cc2500_settings_persistent;


/* betaflight settings API */
// main/config/config.h:
void bf_writeEEPROM(void);
#define writeEEPROM() bf_writeEEPROM()


// main/pg/rx.h:
#define FRAME_ERR_RESAMPLE_US 100000

#define GET_FRAME_ERR_LPF_FREQUENCY(period) (1 / (period / 10.0f))

typedef struct rxConfig_s {
//    uint8_t rcmap[RX_MAPPABLE_CHANNEL_COUNT];  // mapping of radio channels to internal RPYTA+ order
    uint8_t serialrx_provider;              // type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
//    uint8_t serialrx_inverted;              // invert the serial RX protocol compared to it's default setting
//    uint8_t halfDuplex;                     // allow rx to operate in half duplex mode on F4, ignored for F1 and F3.
//    ioTag_t spektrum_bind_pin_override_ioTag;
//    ioTag_t spektrum_bind_plug_ioTag;
//    uint8_t spektrum_sat_bind;              // number of bind pulses for Spektrum satellite receivers
//    uint8_t spektrum_sat_bind_autoreset;    // whenever we will reset (exit) binding mode after hard reboot
    uint8_t rssi_channel;
//    uint8_t rssi_scale;
//    uint8_t rssi_invert;
    uint16_t midrc;                         // Some radios have not a neutral point centered on 1500. can be changed here
//    uint16_t mincheck;                      // minimum rc end
//    uint16_t maxcheck;                      // maximum rc end
//    uint8_t rcInterpolation;
//    uint8_t rcInterpolationChannels;
//    uint8_t rcInterpolationInterval;
//    uint8_t fpvCamAngleDegrees;             // Camera angle to be scaled into rc commands
//    uint8_t airModeActivateThreshold;       // Throttle setpoint percent where airmode gets activated
//
//    uint16_t rx_min_usec;
//    uint16_t rx_max_usec;
    uint8_t max_aux_channel;
//    uint8_t rssi_src_frame_errors;          // true to use frame drop flags in the rx protocol
//    int8_t rssi_offset;                     // offset applied to the RSSI value before it is returned
//    uint8_t rc_smoothing_type;              // Determines the smoothing algorithm to use: INTERPOLATION or FILTER
//    uint8_t rc_smoothing_input_cutoff;      // Filter cutoff frequency for the input filter (0 = auto)
//    uint8_t rc_smoothing_derivative_cutoff; // Filter cutoff frequency for the setpoint weight derivative filter (0 = auto)
//    uint8_t rc_smoothing_debug_axis;        // Axis to log as debug values when debug_mode = RC_SMOOTHING
//    uint8_t rc_smoothing_input_type;        // Input filter type (0 = PT1, 1 = BIQUAD)
//    uint8_t rc_smoothing_derivative_type;   // Derivative filter type (0 = OFF, 1 = PT1, 2 = BIQUAD)
//    uint8_t rc_smoothing_auto_factor;       // Used to adjust the "smoothness" determined by the auto cutoff calculations
    uint8_t rssi_src_frame_lpf_period;      // Period of the cutoff frequency for the source frame RSSI filter (in 0.1 s)
//
//    uint8_t srxl2_unit_id; // Spektrum SRXL2 RX unit id
//    uint8_t srxl2_baud_fast; // Select Spektrum SRXL2 fast baud rate
//    uint8_t sbus_baud_fast; // Select SBus fast baud rate
} rxConfig_t;

const rxConfig_t* rxConfig(void);

// main/pg/rx_spi.h:
typedef struct rxSpiConfig_s {
    // RX protocol
    uint8_t rx_spi_protocol;                // type of SPI RX protocol
//                                            // nrf24: 0 = v202 250kbps. (Must be enabled by FEATURE_RX_NRF24 first.)
//    uint32_t rx_spi_id;
//    uint8_t rx_spi_rf_channel_count;
//
//    // SPI Bus
//    ioTag_t csnTag;
//    uint8_t spibus;
//
    ioTag_t bindIoTag;
    ioTag_t ledIoTag;
    uint8_t ledInversion;

    ioTag_t extiIoTag;
} rxSpiConfig_t;

const rxSpiConfig_t* rxSpiConfig(void);


// main/pg/rx_spi_cc2500.h:
typedef enum {
  FRSKY_SPI_A1_SOURCE_VBAT = 0,
  FRSKY_SPI_A1_SOURCE_EXTADC,
  FRSKY_SPI_A1_SOURCE_CONST
} frSkySpiA1Source_e;

typedef struct rxCc2500SpiConfig_s {
    uint8_t autoBind;
    uint8_t bindTxId[2];
    int8_t  bindOffset;
    uint8_t bindHopData[50];
    uint8_t rxNum;
    uint8_t a1Source;
    uint8_t chipDetectEnabled;
//    ioTag_t txEnIoTag;
//    ioTag_t lnaEnIoTag;
//    ioTag_t antSelIoTag;
} rxCc2500SpiConfig_t;

const rxCc2500SpiConfig_t* rxCc2500SpiConfig(void);
rxCc2500SpiConfig_t* rxCc2500SpiConfigMutable(void);


// main/telemetry/telemetry.h:
typedef struct telemetryConfig_s {
//    int16_t gpsNoFixLatitude;
//    int16_t gpsNoFixLongitude;
//    uint8_t telemetry_inverted;
//    uint8_t halfDuplex;
//    frskyGpsCoordFormat_e frsky_coordinate_format;
//    frskyUnit_e frsky_unit;
//    uint8_t frsky_vfas_precision;
//    uint8_t hottAlarmSoundInterval;
    uint8_t pidValuesAsTelemetry;
    uint8_t report_cell_voltage;
//    uint8_t flysky_sensors[IBUS_SENSOR_COUNT];
//    uint16_t mavlink_mah_as_heading_divisor;
//    uint32_t disabledSensors; // bit flags
} telemetryConfig_t;

const telemetryConfig_t* telemetryConfig(void);

typedef enum {
    SENSOR_VOLTAGE         = 1 << 0,
    SENSOR_CURRENT         = 1 << 1,
    SENSOR_FUEL            = 1 << 2,
    SENSOR_MODE            = 1 << 3,
    SENSOR_ACC_X           = 1 << 4,
    SENSOR_ACC_Y           = 1 << 5,
    SENSOR_ACC_Z           = 1 << 6,
    SENSOR_PITCH           = 1 << 7,
    SENSOR_ROLL            = 1 << 8,
    SENSOR_HEADING         = 1 << 9,
    SENSOR_ALTITUDE        = 1 << 10,
    SENSOR_VARIO           = 1 << 11,
    SENSOR_LAT_LONG        = 1 << 12,
    SENSOR_GROUND_SPEED    = 1 << 13,
    SENSOR_DISTANCE        = 1 << 14,
    ESC_SENSOR_CURRENT     = 1 << 15,
    ESC_SENSOR_VOLTAGE     = 1 << 16,
    ESC_SENSOR_RPM         = 1 << 17,
    ESC_SENSOR_TEMPERATURE = 1 << 18,
    ESC_SENSOR_ALL         = ESC_SENSOR_CURRENT \
                            | ESC_SENSOR_VOLTAGE \
                            | ESC_SENSOR_RPM \
                            | ESC_SENSOR_TEMPERATURE,
    SENSOR_TEMPERATURE     = 1 << 19,
    SENSOR_ALL             = (1 << 20) - 1,
} sensor_e;
#define SENSOR_NONE 0

bool telemetryIsSensorEnabled(sensor_e sensor);


#endif // RADIO_CONTROL_CC2500_SETTINGS_H
