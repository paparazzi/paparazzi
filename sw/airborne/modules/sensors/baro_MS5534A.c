/*
 * Copyright (C) 2007  ENAC
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

/**
 * @file modules/sensors/baro_MS5534A.c
 * @brief Handling of the MS5534a pressure sensor
 *
 * uses: MOSI, MISO, SCK and 32kHz @ P0.7 with 5V for the -A type
 */

#include "modules/sensors/baro_MS5534A.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/uart.h"
#ifndef BARO_NO_DOWNLINK
#include "subsystems/datalink/downlink.h"
#endif
#include "subsystems/abi.h"
#include "firmwares/fixedwing/nav.h"
#include "state.h"


bool baro_MS5534A_do_reset;
uint32_t baro_MS5534A_pressure;
uint16_t baro_MS5534A_temp;
bool baro_MS5534A_available;
bool alt_baro_enabled;
uint32_t baro_MS5534A_ground_pressure;
float baro_MS5534A_r;
float baro_MS5534A_sigma2;
float baro_MS5534A_z;


#define STATUS_INIT1               0
#define STATUS_INIT2               1
#define STATUS_INIT3               2
#define STATUS_INIT4               3
#define STATUS_MEASURE_PRESSURE    4
#define STATUS_MEASURE_TEMPERATURE 5
#define STATUS_RESET               6

static uint8_t status;
static bool status_read_data;
static uint16_t words[4];

#define InitStatus() (status <= STATUS_INIT4)

#define NextStatus() { \
    if (status_read_data) { \
      status++; if (status > STATUS_MEASURE_TEMPERATURE) status = STATUS_MEASURE_PRESSURE; \
    }; \
    status_read_data = !status_read_data; \
  }

#define CMD_INIT        0x1D
#define CMD_MEASUREMENT 0x0F

#define CMD_W1          0x50
#define CMD_W2          0x60
#define CMD_W3          0x90
#define CMD_W4          0xA0
#define CMD_PRESSURE    0x40
#define CMD_TEMPERATURE 0x20
static uint8_t cmds[6] = {CMD_W1, CMD_W2, CMD_W3, CMD_W4, CMD_PRESSURE, CMD_TEMPERATURE};
static uint8_t reset[3] = {0x15, 0x55, 0x40};



static uint8_t buf_input[3];
static uint8_t buf_output[3];

#define Uint16(buf_input) (buf_input[0] << 8 | buf_input[1])

/* PWM prescaler, set PWM input clock to 15MHz, PWM_CLK = PCLK / PWM_PRESCALER */

#if (PCLK == 15000000)
#define PWM_PRESCALER   1
#else

#if (PCLK == 30000000)
#define PWM_PRESCALER   2
#else

#if (PCLK == 60000000)
#define PWM_PRESCALER   4
#else

#error unknown PCLK frequency
#endif
#endif
#endif

#define MS5534A_MCLK 32768
#define PWM_PERIOD ((PCLK / PWM_PRESCALER) / MS5534A_MCLK)
#define PWM_DUTY (PWM_PERIOD / 2)


static void calibration(void);

void baro_MS5534A_init(void)
{
  /* 32768Hz on PWM2 */
  /* Configure P0.7 pin as PWM */
  PINSEL0 &= ~(_BV(14));
  PINSEL0 |= _BV(15);

  /* No prescaler */
  PWMPR = PWM_PRESCALER - 1;

  /* To get 32768Hz from a base frequency of 15MHz */
  PWMMR0 = PWM_PERIOD;
  PWMMR2 = PWM_DUTY;

  PWMLER = PWMLER_LATCH0 | PWMLER_LATCH2;
  PWMTCR = PWMTCR_COUNTER_ENABLE | PWMTCR_PWM_ENABLE;
  PWMPCR = PWMPCR_ENA2;

#ifdef BARO_MS5534A_W1
  words[0] = BARO_MS5534A_W1;
  words[1] = BARO_MS5534A_W2;
  words[2] = BARO_MS5534A_W3;
  words[3] = BARO_MS5534A_W4;

  status = STATUS_MEASURE_PRESSURE;
  status_read_data = false;

  calibration();
#else
  status = STATUS_INIT1;
  status_read_data = false;
#endif



  baro_MS5534A_available = false;
  alt_baro_enabled = false;

  baro_MS5534A_ground_pressure = 101300;
  baro_MS5534A_r = 20.;
  baro_MS5534A_sigma2 = 1;
  baro_MS5534A_do_reset = false;
}

void baro_MS5534A_reset(void)
{
  status = STATUS_INIT1;
  status_read_data = false;
}

/* To be called not faster than 30Hz */
void baro_MS5534A_send(void)
{
  if (!SpiCheckAvailable()) {
    SpiOverRun();
    return;
  }

  if (status == STATUS_RESET) {
    uint8_t i;
    for (i = 0; i < 3; i++) {
      buf_output[i] = reset[i];
    }
    spi_buffer_length = 3;
  } else {
    if (status_read_data) {
      buf_output[0] = buf_output[1] = 0;
    } else {
      buf_output[0] = (InitStatus() ? CMD_INIT : CMD_MEASUREMENT);
      buf_output[1] = cmds[status];
    }
    spi_buffer_length = 2;
  }

  spi_buffer_input = (uint8_t *)&buf_input;
  spi_buffer_output = (uint8_t *)&buf_output;
  if (status_read_data) {
    SpiSetCPHA();
  } else {
    SpiClrCPHA();
  }
  SpiStart();
}

static uint16_t d1, d2;
static uint16_t c1, c2, c3, c4, ut1, c6;


static void calibration(void)
{
  /* End of init, configuration (page 11) */
  c1 = words[0] >> 1;
  c2 = ((words[2] & 0x3f) << 6) | (words[3] & 0x3f);
  c3 = words[3] >> 6;
  c4 = words[2] >> 6;
  uint16_t c5 = ((words[0] & 0x1) << 10) | (words[1] >> 6);
  c6 = words[1] & 0x3f;

  ut1 = (c5 << 3) + 20224;

#ifndef BARO_NO_DOWNLINK
  DOWNLINK_SEND_BARO_WORDS(DefaultChannel, DefaultDevice, &words[0], &words[1], &words[2], &words[3]);
#endif
}



/* Handle the SPI message, i.e. store the received values in variables */
void baro_MS5534A_event_task(void)
{
  if (status_read_data) {
    switch (status) {
      case STATUS_MEASURE_PRESSURE:
        d1 = Uint16(buf_input);
        break;
      case STATUS_MEASURE_TEMPERATURE:
        d2 = Uint16(buf_input);
        /* Compute pressure and temp (page 10) */
        int16_t dT = d2 - ut1;
        baro_MS5534A_temp = 200 + (dT * (c6 + 50)) / (1 << 10);
        int16_t off = c2 * 4 + ((c4 - 512) * dT) / (1 << 12);
        uint32_t sens = c1 + (c3 * dT) / (1 << 10) + 24576;
        uint16_t x = (sens * (d1 - 7168)) / (1 << 14) - off;
        // baro_MS5534A = ((x*10)>>5) + 250*10;
        baro_MS5534A_pressure = ((x * 100) >> 5) + 250 * 100;
        baro_MS5534A_available = true;

        break;
      case STATUS_RESET:
        break;
      default: /* Init status */
        words[status] = Uint16(buf_input);
        if (status == STATUS_INIT4) {
          calibration();
        }
    }
  } /* else nothing to read */

  NextStatus();
  if (!status_read_data) {
    /* Ask next conversion now */
    baro_MS5534A_send();
  }
}

void baro_MS5534A_event(void)
{
  if (spi_message_received) {
    /* Got a message on SPI. */
    spi_message_received = false;
    baro_MS5534A_event_task();
    if (baro_MS5534A_available) {
      uint32_t now_ts = get_sys_time_usec();
      baro_MS5534A_available = false;
      baro_MS5534A_z = ground_alt + ((float)baro_MS5534A_ground_pressure - baro_MS5534A_pressure) * 0.084;
#if SENSO_SYNC_SEND
      DOWNLINK_SEND_BARO_MS5534A(DefaultChannel, DefaultDevice, &baro_MS5534A_pressure, &baro_MS5534A_temp, &baro_MS5534A_z);
#endif
      float pressure = (float)baro_MS5534A_pressure;
      AbiSendMsgBARO_ABS(BARO_MS5534A_SENDER_ID, now_ts, pressure);
    }
  }
}
