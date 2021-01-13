/*
 * Copyright (C) 2018 Alexandre Bustico <alexandre.bustico@enac.fr>
 *                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * @file    esc_dshot.c
 * @brief   DSHOT driver based on ChibiOS
 *
 * @author Alexandre Bustico
 * @maintainer Gautier Hattenberger <gautier.hattenberger@enac.fr>
 */

#include "modules/actuators/esc_dshot.h"
#include <stdnoreturn.h>
#include <math.h>
#include <string.h>

/*
#                     _            __   _            _    _      _
#                    | |          / _| (_)          (_)  | |    (_)
#                  __| |    ___  | |_   _    _ __    _   | |_    _     ___    _ __
#                 / _` |   / _ \ |  _| | |  | '_ \  | |  | __|  | |   / _ \  | '_ \
#                | (_| |  |  __/ | |   | |  | | | | | |  \ |_   | |  | (_) | | | | |
#                 \__,_|   \___| |_|   |_|  |_| |_| |_|   \__|  |_|   \___/  |_| |_|
*/

/** Base freq of DSHOT signal (in kHz)
 * Possible values are: 150, 300, 600
 */
#ifndef DSHOT_SPEED
#define DSHOT_SPEED 300
#endif

/** Baudrate of the serial link used for telemetry data
 * Can depend on the ESC, but only 115k have been used so far
 */
#ifndef DSHOT_TELEMETRY_BAUD
#define DSHOT_TELEMETRY_BAUD 115200
#endif

/** Telemetry timeout in ms
 */
#ifndef DSHOT_TELEMETRY_TIMEOUT_MS
#define DSHOT_TELEMETRY_TIMEOUT_MS 3
#endif

/** the timer will beat @84Mhz on STM32F4 // TODO check on F7 */
#define PWM_FREQ (STM32_SYSCLK/2000)

/** Ticks per period
 * that let use any timer:
 * does not care if linked to PCLK1 or PCLK2
 * tick_per_period will be dynamically calculated
 * after pwm init
 */
#define TICKS_PER_PERIOD 1000

// Some extra defines and macros
#define DSHOT_FREQ (DSHOT_SPEED*1000) // in Hz
#define TICK_FREQ (PWM_FREQ * TICKS_PER_PERIOD)
#define DSHOT_PWM_PERIOD (TICK_FREQ/DSHOT_FREQ)
#define DSHOT_BIT0_DUTY (DSHOT_PWM_PERIOD * 373 / 1000)
#define DSHOT_BIT1_DUTY (DSHOT_BIT0_DUTY*2)
#define DCR_DBL ((DSHOT_CHANNELS-1) << 8) //  DSHOT_CHANNELS transfert(s), first register to get is CCR1
#define DCR_DBA(pwmd) (((uint32_t *) (&pwmd->tim->CCR) - ((uint32_t *) pwmd->tim)))

#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))
#define Min(x,y) (x < y ? x : y)

#define DSHOT_MAX_VALUE ((1<<11)-1) // 11 bits used to send command, so maximum value is 2047

/*
#                 _ __                   _              _      _   _    _ __
#                | '_ \                 | |            | |    | | | |  | '_ \
#                | |_) |  _ __    ___   | |_     ___   | |_   | |_| |  | |_) |   ___
#                | .__/  | '__|  / _ \  | __|   / _ \  | __|   \__, |  | .__/   / _ \
#                | |     | |    | (_) | \ |_   | (_) | \ |_     __/ |  | |     |  __/
#                |_|     |_|     \___/   \__|   \___/   \__|   |___/   |_|      \___|
*/
static DshotPacket makeDshotPacket(const uint16_t throttle, const bool tlmRequest);
static inline void setDshotPacketThrottle(DshotPacket * const dp, const uint16_t throttle);
static inline void setDshotPacketTlm(DshotPacket * const dp, const bool tlmRequest);
static void buildDshotDmaBuffer(DshotPackets * const dsp,  DshotDmaBuffer * const dma, const size_t timerWidth);
static inline uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed);
static uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen);
static noreturn void dshotTlmRec(void *arg);
static size_t getTimerWidth(const PWMDriver *pwmp);

/*
#                         _ __    _
#                        | '_ \  (_)
#                  __ _  | |_) |  _
#                 / _` | | .__/  | |
#                | (_| | | |     | |
#                 \__,_| |_|     |_|
*/

/**
 * @brief   Configures and activates the DSHOT peripheral.
 *
 * @param[in] driver    pointer to the @p DSHOTDriver object
 * @param[in] config    pointer to the @p DSHOTConfig object.
 * @api
 */
void dshotStart(DSHOTDriver *driver, const DSHOTConfig *config)
{
  memset((void *) config->dma_buf, 0, sizeof(*(config->dma_buf)));
  const size_t timerWidthInBytes = getTimerWidth(config->pwmp);

  static const SerialConfig  tlmcfg =  {
    .speed = DSHOT_TELEMETRY_BAUD,
    .cr1 = 0,                                      // pas de parité
    .cr2 = USART_CR2_STOP1_BITS,       // 1 bit de stop
    .cr3 = 0                                       // pas de controle de flux hardware (CTS, RTS)
  };

  driver->config = config;
  // use pburst, mburst only if buffer size satisfy aligmnent requirement
  driver->dma_conf = (DMAConfig) {
    .stream = config->dma_stream,
    .channel = config->dma_channel,
    .dma_priority = 3,
    .irq_priority = 2,
    .direction = DMA_DIR_M2P,
    .psize = timerWidthInBytes,
    .msize = timerWidthInBytes,
#if __DCACHE_PRESENT
    .dcache_memory_in_use = config->dcache_memory_in_use,
#endif
    .inc_peripheral_addr = false,
    .inc_memory_addr = true,
    .circular = false,
    .error_cb = NULL,
    .end_cb = NULL,
    .pburst = 0,
    .mburst = 0,
    .fifo = 4
  };

  driver->pwm_conf = (PWMConfig) {
  .frequency = TICK_FREQ,
  .period    = TICKS_PER_PERIOD,
  .callback  = NULL,
  .channels  = {
    {.mode = PWM_OUTPUT_ACTIVE_HIGH,
     .callback = NULL},
    {.mode = DSHOT_CHANNELS > 1 ? PWM_OUTPUT_ACTIVE_HIGH : PWM_OUTPUT_DISABLED,
     .callback = NULL},
    {.mode = DSHOT_CHANNELS > 2 ? PWM_OUTPUT_ACTIVE_HIGH : PWM_OUTPUT_DISABLED,
     .callback = NULL},
    {.mode = DSHOT_CHANNELS > 3 ? PWM_OUTPUT_ACTIVE_HIGH : PWM_OUTPUT_DISABLED,
     .callback = NULL},
  },
  .cr2  =  STM32_TIM_CR2_CCDS,
  .dier =  STM32_TIM_DIER_UDE
  };

  driver->crc_errors = 0;
  dmaObjectInit(&driver->dmap);
  chMBObjectInit(&driver->mb, driver->_mbBuf, ARRAY_LEN(driver->_mbBuf));

  const bool dmaOk = dmaStart(&driver->dmap, &driver->dma_conf);
  chDbgAssert(dmaOk == true, "dshot dma start error");

  if (driver->config->tlm_sd) {
    sdStart(driver->config->tlm_sd, &tlmcfg);
    chThdCreateStatic(driver->waDshotTlmRec, sizeof(driver->waDshotTlmRec), NORMALPRIO,
                      dshotTlmRec, driver);
  }

  pwmStart(driver->config->pwmp, &driver->pwm_conf);
  driver->config->pwmp->tim->DCR = DCR_DBL | DCR_DBA(driver->config->pwmp); // enable bloc register DMA transaction
  pwmChangePeriod(driver->config->pwmp, DSHOT_PWM_PERIOD);

  for (size_t j = 0; j < DSHOT_CHANNELS; j++) {
    pwmEnableChannel(driver->config->pwmp, j, 0);
    driver->dshotMotors.dp[j] =  makeDshotPacket(0, 0);
  }
  driver->dshotMotors.onGoingQry = false;
  driver->dshotMotors.currentTlmQry = 0U;
}

/**
 * @brief   prepare throttle order for specified ESC
 *
 * @param[in] driver    pointer to the @p DSHOTDriver object
 * @param[in] index     channel : [0..3] or [0..1] depending on driver used
 * @param[in] throttle  [48 .. 2047]
 * @note      dshotSendFrame has to be called after using this function
 * @note      see also dshotSendThrottles
 * @api
 */
void dshotSetThrottle(DSHOTDriver *driver, const uint8_t index,
                      const  uint16_t throttle)
{
  if (throttle > 0 && throttle <= DSHOT_CMD_MAX) {
    chDbgAssert(false, "dshotSetThrottle throttle error");
    return; // special commands (except MOTOR_STOP) can't be applied from this function
  } else {
    // send normal throttle
    if (index == DSHOT_ALL_MOTORS) {
      for (uint8_t _index = 0; _index < DSHOT_CHANNELS; _index++) {
        setDshotPacketThrottle(&driver->dshotMotors.dp[_index], Min(throttle, DSHOT_MAX_VALUE));
      }
    } else if ((index - DSHOT_CHANNEL_FIRST_INDEX) < DSHOT_CHANNELS) {
      setDshotPacketThrottle(&driver->dshotMotors.dp[index - DSHOT_CHANNEL_FIRST_INDEX],
			     Min(throttle, DSHOT_MAX_VALUE));
    } else {
      chDbgAssert(false, "dshotSetThrottle index error");
    }
  }
}

/**
 * @brief   send special order to one of the ESC (BHELIX, KISS, ...)
 *
 * @param[in] driver    pointer to the @p DSHOTDriver object
 * @param[in] index     channel : [0..3] or [0..1] depending on driver used
 * @param[in] specmd   special commands, see enum
 * @api
 */
void dshotSendSpecialCommand(DSHOTDriver *driver, const  uint8_t index,
                             const dshot_special_commands_t specmd)
{
  if (specmd > DSHOT_CMD_MAX) {
    return; // Don't apply special commands from this function
  }
  if (index < DSHOT_CHANNELS) {
    setDshotPacketThrottle(&driver->dshotMotors.dp[index], specmd);
    setDshotPacketTlm(&driver->dshotMotors.dp[index], driver->config->tlm_sd != NULL);
  } else if (index == DSHOT_ALL_MOTORS) {
    for (uint8_t _index = 0; _index < DSHOT_CHANNELS; _index++) {
      setDshotPacketThrottle(&driver->dshotMotors.dp[_index], specmd);
      setDshotPacketTlm(&driver->dshotMotors.dp[_index], driver->config->tlm_sd != NULL);
    }
  }

  uint8_t repeat;
  switch (specmd) {
    case DSHOT_CMD_SPIN_DIRECTION_1:
    case DSHOT_CMD_SPIN_DIRECTION_2:
    case DSHOT_CMD_3D_MODE_OFF:
    case DSHOT_CMD_3D_MODE_ON:
    case DSHOT_CMD_SAVE_SETTINGS:
    case DSHOT_CMD_SETTINGS_REQUEST:
    case DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF:
    case DSHOT_CMD_SILENT_MODE_ON_OFF:
      repeat = 10;
      break;
    default:
      repeat = 1;
  }

  while (repeat--) {
    dshotSendFrame(driver);
    chThdSleepMilliseconds(1);
  }
}

/**
 * @brief   send throttle packed order to all of the ESCs
 *
 * @param[in] driver    pointer to the @p DSHOTDriver object
 * @param[in] throttle[DSHOT_CHANNELS]  [48 .. 2047]
 * @note dshotSendFrame is called by this function
 * @note telemetry bit is set in turn for each ESC of the ESCs
 * @api
 */
void dshotSendThrottles(DSHOTDriver *driver, const  uint16_t throttles[DSHOT_CHANNELS])
{
  for (uint8_t index = 0; index < DSHOT_CHANNELS; index++) {
    setDshotPacketThrottle(&driver->dshotMotors.dp[index], throttles[index]);
  }

  dshotSendFrame(driver);
}

/**
 * @brief   send throttle  order
 *
 * @param[in] driver    pointer to the @p DSHOTDriver object
 * @note dshotSetXXX api should be called prior to this function
 * @api
 */
void dshotSendFrame(DSHOTDriver *driver)
{
  if (driver->dmap.state == DMA_READY) {
    if ((driver->config->tlm_sd != NULL) &&
        (driver->dshotMotors.onGoingQry == false)) {
      driver->dshotMotors.onGoingQry = true;
      const uint32_t index = (driver->dshotMotors.currentTlmQry + 1) % DSHOT_CHANNELS;
      driver->dshotMotors.currentTlmQry = index;
      setDshotPacketTlm(&driver->dshotMotors.dp[index], true);
      chMBPostTimeout(&driver->mb, driver->dshotMotors.currentTlmQry, TIME_IMMEDIATE);
    }

    buildDshotDmaBuffer(&driver->dshotMotors, driver->config->dma_buf, getTimerWidth(driver->config->pwmp));
    dmaStartTransfert(&driver->dmap,
                      &driver->config->pwmp->tim->DMAR,
                      driver->config->dma_buf, DSHOT_DMA_BUFFER_SIZE * DSHOT_CHANNELS);

  }
}

/**
 * @brief   return number of telemetry crc error since dshotStart
 *
 * @param[in] driver    pointer to the @p DSHOTDriver object
 * @return    number of CRC errors
 * @api
 */
uint32_t dshotGetCrcErrorsCount(DSHOTDriver *driver)
{
  return driver->crc_errors;
}

/**
 * @brief   return last received telemetry data
 *
 * @param[in] driver    pointer to the @p DSHOTDriver object
 * @param[in] index     channel : [0..3] or [0..1] depending on driver used
 * @return    pointer on a telemetry structure
 * @api
 */
const DshotTelemetry *dshotGetTelemetry(const DSHOTDriver *driver, const uint32_t index)
{
  return &driver->dshotMotors.dt[index];
}


/*
#                 _ __           _                    _
#                | '_ \         (_)                  | |
#                | |_) |  _ __   _   __   __   __ _  | |_     ___
#                | .__/  | '__| | |  \ \ / /  / _` | | __|   / _ \
#                | |     | |    | |   \ V /  | (_| | \ |_   |  __/
#                |_|     |_|    |_|    \_/    \__,_|  \__|   \___|
*/

static DshotPacket makeDshotPacket(const uint16_t _throttle, const bool tlmRequest)
{
  DshotPacket dp = {.throttle = _throttle,
                    .telemetryRequest = (tlmRequest ? 1 : 0),
                    .crc = 0
                   };

  // compute checksum
  uint16_t csum = (_throttle << 1) | dp.telemetryRequest;
  for (int i = 0; i < 3; i++) {
    dp.crc ^=  csum;   // xor data by nibbles
    csum >>= 4;
  }

  return dp;
}

static inline void setDshotPacketThrottle(DshotPacket *const dp, const uint16_t throttle)
{
  dp->throttle = throttle;
  dp->telemetryRequest = 0;
}

static inline void setDshotPacketTlm(DshotPacket *const dp, const bool tlmRequest)
{
  dp->telemetryRequest =  tlmRequest ? 1 : 0;
}

static void buildDshotDmaBuffer(DshotPackets *const dsp, DshotDmaBuffer *const dma, const size_t timerWidth)
{
  for (size_t chanIdx = 0; chanIdx < DSHOT_CHANNELS; chanIdx++) {
    // compute checksum
    DshotPacket *const dp = &dsp->dp[chanIdx];
    dp->crc = 0;
    uint16_t csum = (dp->throttle << 1) | dp->telemetryRequest;
    for (int i = 0; i < 3; i++) {
      dp->crc ^=  csum;   // xor data by nibbles
      csum >>= 4;
    }
    // generate pwm frame
    for (size_t bitIdx = 0; bitIdx < DSHOT_BIT_WIDTHS; bitIdx++) {
      const uint16_t value = dp->rawFrame &
                             (1 << ((DSHOT_BIT_WIDTHS - 1) - bitIdx)) ?
                             DSHOT_BIT1_DUTY : DSHOT_BIT0_DUTY;
      if (timerWidth == 2) {
        dma->widths16[bitIdx+DSHOT_PRE_FRAME_SILENT_SYNC_BITS][chanIdx] = value;
      } else {
#if DSHOT_AT_LEAST_ONE_32B_TIMER
        dma->widths32[bitIdx+DSHOT_PRE_FRAME_SILENT_SYNC_BITS][chanIdx] = value;
#else
        chSysHalt("use of 32 bit timer implies to define DSHOT_AT_LEAST_ONE_32B_TIMER to TRUE");
#endif
      }
    }
    // the bits for silence sync (pre and post) in case of continous sending are zeroed once at init
  }
}

static inline uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed)
{
  uint8_t crc_u = crc;
  crc_u ^= crc_seed;

  for (int i = 0; i < 8; i++) {
    crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
  }

  return (crc_u);
}

static uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen)
{
  uint8_t crc = 0;
  for (int i = 0; i < BufLen; i++) {
    crc = updateCrc8(Buf[i], crc);
  }

  return crc;
}

__attribute__((const))
static size_t   getTimerWidth(const PWMDriver *pwmp)
{
  (void) pwmp;

  return (0
#if STM32_PWM_USE_TIM2
          || (pwmp == &PWMD2)
#endif
#if STM32_PWM_USE_TIM5
          || (pwmp == &PWMD5)
#endif
         ) ? 4 : 2;
}


/*
#                 _      _                                 _
#                | |    | |                               | |
#                | |_   | |__    _ __    ___    __ _    __| |   ___
#                | __|  | '_ \  | '__|  / _ \  / _` |  / _` |  / __|
#                \ |_   | | | | | |    |  __/ | (_| | | (_| |  \__ \
#                 \__|  |_| |_| |_|     \___|  \__,_|  \__,_|  |___/
*/

static noreturn void dshotTlmRec(void *arg)
{
  DSHOTDriver *driver = (DSHOTDriver *) arg;

  uint32_t escIdx = 0;

  chRegSetThreadName("dshotTlmRec");
  while (true) {
    chMBFetchTimeout(&driver->mb, (msg_t *) &escIdx, TIME_INFINITE);
    const uint32_t idx = escIdx;
    const bool success =
      (sdReadTimeout(driver->config->tlm_sd, driver->dshotMotors.dt[idx].rawData, sizeof(DshotTelemetry),
                     TIME_MS2I(DSHOT_TELEMETRY_TIMEOUT_MS)) == sizeof(DshotTelemetry));
    if (!success ||
        (calculateCrc8(driver->dshotMotors.dt[idx].rawData,
                       sizeof(driver->dshotMotors.dt[idx].rawData)) != driver->dshotMotors.dt[idx].crc8)) {
      // empty buffer to resync
      while (sdGetTimeout(driver->config->tlm_sd, TIME_IMMEDIATE) >= 0) {};
      memset(driver->dshotMotors.dt[idx].rawData, 0U, sizeof(DshotTelemetry));
      // count errors
      driver->crc_errors++;
    }
    driver->dshotMotors.onGoingQry = false;
  }
}

