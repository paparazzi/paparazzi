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

/*
 * This file contains compatibility code for the betaflight cc2500 frsky
 * drivers.
 *
 * Notes:
 * - Function macros are used so that betaflight names can be used when this
 *   header is included, while the actual functions can have a bf_ prefix to
 *   prevent possible name collisions.
 */

#ifndef RADIO_CONTROL_CC2500_COMPAT_H
#define RADIO_CONTROL_CC2500_COMPAT_H


#pragma GCC diagnostic ignored "-Wmissing-prototypes"
#pragma GCC diagnostic ignored "-Wstrict-prototypes"
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wshadow"


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define USE_RX_SPI
#define USE_RX_FRSKY_SPI
#define USE_RX_FRSKY_SPI_TELEMETRY

#if (CC2500_RX_SPI_PROTOCOL == RX_SPI_FRSKY_X_LBT) || (CC2500_RX_SPI_PROTOCOL == RX_SPI_FRSKY_X)
#define USE_RX_FRSKY_SPI_X
#define USE_TELEMETRY_SMARTPORT
#endif
#if (CC2500_RX_SPI_PROTOCOL == RX_SPI_FRSKY_D)
#define USE_RX_FRSKY_SPI_D
#endif

#define DEBUG_SET(...) /* Do nothing */
#define STATIC_ASSERT(...) /* Do nothing */
#define STATIC_UNIT_TESTED static


// (unknown):
#define sensors(...) 1

struct attitude_values_t {
  int8_t roll;
  int8_t pitch;
  int8_t yaw;
};
struct attitude_t {
  struct attitude_values_t values;
};
extern struct attitude_t bf_attitude;
#define attitude bf_attitude


// main/common/utils.h:
#if __GNUC__ > 6
#define FALLTHROUGH __attribute__ ((fallthrough))
#else
#define FALLTHROUGH do {} while(0)
#endif


// main/common/time.h:
typedef int32_t timeDelta_t;
typedef uint32_t timeMs_t ;
typedef uint32_t timeUs_t;
#define TIMEUS_MAX UINT32_MAX

static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) { return (timeDelta_t)(a - b); }


// main/common/maths.h:
#define MIN(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })


// main/common/filter.h:
typedef struct pt1Filter_s {
    float state;
    float k;
} pt1Filter_t;

float pt1FilterGain(float f_cut, float dT);
void pt1FilterInit(pt1Filter_t *filter, float k);
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k);
float pt1FilterApply(pt1Filter_t *filter, float input);


// main/config/config.h:
#define PID_ROLL 0
#define PID_PITCH 0
#define PID_YAW 0

struct pidGains_s {
  uint8_t P;
  uint8_t I;
  uint8_t D;
};
struct pidProfile_s {
  struct pidGains_s pid[1];
};
extern struct pidProfile_s *currentPidProfile;


// main/config/feature.h:
typedef enum {
    FEATURE_RX_PPM = 1 << 0,
//    FEATURE_INFLIGHT_ACC_CAL = 1 << 2,
    FEATURE_RX_SERIAL = 1 << 3,
//    FEATURE_MOTOR_STOP = 1 << 4,
//    FEATURE_SERVO_TILT = 1 << 5,
//    FEATURE_SOFTSERIAL = 1 << 6,
//    FEATURE_GPS = 1 << 7,
//    FEATURE_RANGEFINDER = 1 << 9,
    FEATURE_TELEMETRY = 1 << 10,
//    FEATURE_3D = 1 << 12,
    FEATURE_RX_PARALLEL_PWM = 1 << 13,
    FEATURE_RX_MSP = 1 << 14,
    FEATURE_RSSI_ADC = 1 << 15,
//    FEATURE_LED_STRIP = 1 << 16,
//    FEATURE_DASHBOARD = 1 << 17,
//    FEATURE_OSD = 1 << 18,
//    FEATURE_CHANNEL_FORWARDING = 1 << 20,
//    FEATURE_TRANSPONDER = 1 << 21,
//    FEATURE_AIRMODE = 1 << 22,
    FEATURE_RX_SPI = 1 << 25,
//    //FEATURE_SOFTSPI = 1 << 26, (removed)
//    FEATURE_ESC_SENSOR = 1 << 27,
//    FEATURE_ANTI_GRAVITY = 1 << 28,
//    FEATURE_DYNAMIC_FILTER = 1 << 29,
} features_e;

bool bf_featureIsEnabled(const uint32_t mask);
#define featureIsEnabled(mask) bf_featureIsEnabled(mask)


// main/drivers/time.h:
void bf_delayMicroseconds(timeUs_t us);
#define delayMicroseconds(us) bf_delayMicroseconds(us)

void bf_delay(timeMs_t ms);
#define delay(ms) bf_delay(ms)

timeUs_t bf_micros(void);
#define micros() bf_micros()
timeMs_t bf_millis(void);
#define millis() bf_millis()


// main/drivers/adc.h:
#define ADC_EXTERNAL1 1
uint16_t bf_adcGetChannel(uint8_t channel);
#define adcGetChannel(channel) bf_adcGetChannel(channel)


// main/drivers/rx/rx_spi.h:
#define RX_SPI_MAX_PAYLOAD_SIZE 35

bool bf_rxSpiDeviceInit(void);
#define rxSpiDeviceInit(rxSpiConfig) bf_rxSpiDeviceInit()


// main/drivers/rx/rx_pwm.h:
#define PPM_RCVR_TIMEOUT 0


// main/drivers/io.h:
typedef void(*gpiofnptr_t)(uint32_t port, uint16_t pin);

struct gpio_t {
  uint32_t port;
  uint16_t pin;
  gpiofnptr_t hi;
  gpiofnptr_t lo;
};
typedef struct gpio_t *IO_t;
typedef IO_t ioTag_t;
#define IO_NONE NULL

IO_t bf_IOGetByTag(ioTag_t io);
#define IOGetByTag(io) bf_IOGetByTag(io)

void bf_IOInit(IO_t io, uint8_t owner, uint8_t index);
#define IOInit(io, owner, index) bf_IOInit(io, owner, index)

enum ioconfig_t {
  IOCFG_OUT_PP,
  IOCFG_IN_FLOATING,
  IOCFG_IPU,
};
void bf_IOConfigGPIO(IO_t io, enum ioconfig_t cfg);
#define IOConfigGPIO(io, cfg) bf_IOConfigGPIO(io, cfg)

bool bf_IORead(IO_t gpio);
#define IORead(gpio) bf_IORead(gpio)

void bf_IOHi(IO_t io);
#define IOHi(io) bf_IOHi(io)
void bf_IOLo(IO_t io);
#define IOLo(io) bf_IOLo(io)
void bf_IOToggle(IO_t io);
#define IOToggle(io) bf_IOToggle(io)


// main/fc/runtime_config.h:
#define isArmingDisabled() 0
#define ARMING_FLAG(...) 1
#define FLIGHT_MODE(...) 0


// main/fc/controlrate_profile.h:
#define FD_ROLL 0
#define FD_PITCH 0
#define FD_YAW 0
typedef struct {
  uint8_t rates[1];
} controlRateConfig_t;
extern controlRateConfig_t *currentControlRateProfile;


// main/flight/position.h:
int32_t bf_getEstimatedAltitudeCm(void);
#define getEstimatedAltitudeCm() bf_getEstimatedAltitudeCm()

int16_t bf_getEstimatedVario(void);
#define getEstimatedVario() bf_getEstimatedVario()


// main/drivers/resources.h:
typedef enum {
    OWNER_RX_SPI_EXTI,
    OWNER_RX_SPI_BIND,
    OWNER_LED,
} resourceOwner_e;


// main/sensors/battery.h
bool bf_isBatteryVoltageConfigured(void);
#define isBatteryVoltageConfigured() bf_isBatteryVoltageConfigured()

uint16_t bf_getLegacyBatteryVoltage(void);
#define getLegacyBatteryVoltage() bf_getLegacyBatteryVoltage()
uint16_t bf_getBatteryVoltage(void);
#define getBatteryVoltage() bf_getBatteryVoltage()

uint8_t bf_getBatteryCellCount(void);
#define getBatteryCellCount() bf_getBatteryCellCount()

bool bf_isAmperageConfigured(void);
#define isAmperageConfigured() bf_isAmperageConfigured()
int32_t bf_getAmperage(void);
#define getAmperage() bf_getAmperage()
int32_t bf_getMAhDrawn(void);
#define getMAhDrawn() bf_getMAhDrawn()


#endif // RADIO_CONTROL_CC2500_COMPAT_H
