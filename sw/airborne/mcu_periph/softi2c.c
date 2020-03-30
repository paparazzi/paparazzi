/*
 * Copyright (C) 2020 Tom van Dijk <tomvand@users.noreply.github.com>
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
 * @file mcu_periph/softi2c.c
 * Platform-independent software I2C implementation.
 * Can be used transparently in place of the hardware I2C in i2c.h.
 *
 * This implementation can only be used as the (only) master on the I2C bus.
 */

#include "mcu_periph/softi2c.h"

#include "mcu_arch.h"
#include "mcu_periph/gpio.h"
#include "mcu_periph/sys_time.h"

#include <stdbool.h>

// Number of times to update the softI2C devices per event call.
// Increase for a higher bitrate at the cost of a longer runtime.
// For reference (STM32F4, SYS_TIME_FREQUENCY = 204800Hz):
// - a single device_event function call takes approx. 5 microseconds.
// - with an event multiplier of 1, the maximum bitrate is ~10 kbps
// - while spinning (very large multiplier), the maximum bitrate is ~45 kbps.
#ifndef SOFTI2C_EVENT_MULTIPLIER
#define SOFTI2C_EVENT_MULTIPLIER 1
#endif


struct softi2c_device {
  struct i2c_periph *periph;
  uint32_t sda_port;
  uint16_t sda_pin;
  uint32_t scl_port;
  uint16_t scl_pin;
  float t_scl;  // Clock period
  /* Bit-level state machine */
  uint8_t bit_state;
  float bit_state_time;
  float bit_start_time;
  /* Byte-level state machine */
  uint8_t byte_state;
  /* Transaction-level state machine */
  // periph->status
  // periph->idx_buf
};

struct highres_clock {
  // The SYS_TIME clock resolution is too coarse.
  // Count the number of event calls for a finegrained time estimate.
  float now;
  float last_sys_time;
  float time_per_event;
  uint16_t event_cnt;
};
static struct highres_clock time;


static bool softi2c_idle(struct i2c_periph *periph) __attribute__((unused));
static bool softi2c_submit(struct i2c_periph *periph, struct i2c_transaction *t) __attribute__((unused));
static void softi2c_setbitrate(struct i2c_periph *periph, int bitrate) __attribute__((unused));


#if USE_SOFTI2C0 && ( \
  !defined(SOFTI2C0_SDA_GPIO) || !defined(SOFTI2C0_SDA_PIN) || \
  !defined(SOFTI2C0_SCL_GPIO) || !defined(SOFTI2C0_SCL_PIN))
#error "SDA and SCL pins must be configured for SOFTI2C0!"
#endif

#if USE_SOFTI2C1 && ( \
  !defined(SOFTI2C1_SDA_GPIO) || !defined(SOFTI2C1_SDA_PIN) || \
  !defined(SOFTI2C1_SCL_GPIO) || !defined(SOFTI2C1_SCL_PIN))
#error "SDA and SCL pins must be configured for SOFTI2C1!"
#endif


#if USE_SOFTI2C0
struct i2c_periph softi2c0;
static struct softi2c_device softi2c0_device;
void softi2c0_init(void) {
  i2c_init(&softi2c0);
  softi2c0_hw_init();
}
#endif /* USE_SOFTI2C0 */

#if USE_SOFTI2C1
struct i2c_periph softi2c1;
static struct softi2c_device softi2c1_device;
void softi2c1_init(void) {
  i2c_init(&softi2c1);
  softi2c1_hw_init();
}
#endif /* USE_SOFTI2C1 */


static void softi2c_gpio_highz(uint32_t port, uint16_t pin) {
#if defined(CHIBIOS_MCU_ARCH_H) || defined(STM32_MCU_ARCH_H)
  /* Arch's with input_pullup */
  gpio_setup_input_pullup(port, pin);
#else
  /* Arch's without input_pullup */
  // Gpio_set might enable pullup on some architectures (e.g. arduino), not sure
  // if it works here. If not, an external pull-up resistor is required on the
  // I2C lines.
  gpio_setup_input(port, pin);
  gpio_set(port, pin);
#endif /* arch with/without input_pullup */
}

static bool softi2c_gpio_read(uint32_t port, uint16_t pin) {
  softi2c_gpio_highz(port, pin);
  return gpio_get(port, pin);
}

static void softi2c_gpio_drive_low(uint32_t port, uint16_t pin) {
  gpio_setup_output(port, pin);
  gpio_clear(port, pin);
}

static void softi2c_setup_gpio(
    uint32_t sda_port, uint16_t sda_pin,
    uint32_t scl_port, uint16_t scl_pin) __attribute__((unused));

static void softi2c_setup_gpio(
    uint32_t sda_port, uint16_t sda_pin,
    uint32_t scl_port, uint16_t scl_pin) {
#ifdef STM32_MCU_ARCH_H
  gpio_enable_clock(sda_port);
  gpio_enable_clock(scl_port);
#endif /* STM32F1/F4 */
  softi2c_gpio_highz(sda_port, sda_pin);
  softi2c_gpio_highz(scl_port, scl_pin);
}


#if USE_SOFTI2C0
#ifndef SOFTI2C0_CLOCK_SPEED
#define SOFTI2C0_CLOCK_SPEED 1000
#endif
PRINT_CONFIG_VAR(SOFTI2C0_CLOCK_SPEED);

struct i2c_errors softi2c0_errors;

void softi2c0_hw_init(void) {
  softi2c0.idle = softi2c_idle;
  softi2c0.submit = softi2c_submit;
  softi2c0.setbitrate = softi2c_setbitrate;
  softi2c0.reg_addr = (void *) &softi2c0_device;
  softi2c0.errors = &softi2c0_errors;
  ZEROS_ERR_COUNTER(softi2c0_errors);

  softi2c0_device.periph = &softi2c0;
  softi2c0_device.sda_port = SOFTI2C0_SDA_GPIO;
  softi2c0_device.sda_pin = SOFTI2C0_SDA_PIN;
  softi2c0_device.scl_port = SOFTI2C0_SCL_GPIO;
  softi2c0_device.scl_pin = SOFTI2C0_SCL_PIN;

  /* Set up GPIO */
  softi2c_setup_gpio(
      SOFTI2C0_SDA_GPIO, SOFTI2C0_SDA_PIN,
      SOFTI2C0_SCL_GPIO, SOFTI2C0_SCL_PIN);

  /* Set default bitrate */
  i2c_setbitrate(&softi2c0, SOFTI2C0_CLOCK_SPEED);
}
#endif /* USE_SOFTI2C0 */

#if USE_SOFTI2C1
#ifndef SOFTI2C1_CLOCK_SPEED
#define SOFTI2C1_CLOCK_SPEED 1000
#endif
PRINT_CONFIG_VAR(SOFTI2C1_CLOCK_SPEED);

struct i2c_errors softi2c1_errors;

void softi2c1_hw_init(void) {
  softi2c1.idle = softi2c_idle;
  softi2c1.submit = softi2c_submit;
  softi2c1.setbitrate = softi2c_setbitrate;
  softi2c1.reg_addr = (void *) &softi2c1_device;
  softi2c1.errors = &softi2c1_errors;
  ZEROS_ERR_COUNTER(softi2c1_errors);

  softi2c1_device.periph = &softi2c1;
  softi2c1_device.sda_port = SOFTI2C1_SDA_GPIO;
  softi2c1_device.sda_pin = SOFTI2C1_SDA_PIN;
  softi2c1_device.scl_port = SOFTI2C1_SCL_GPIO;
  softi2c1_device.scl_pin = SOFTI2C1_SCL_PIN;

  /* Set up GPIO */
  softi2c_setup_gpio(
      SOFTI2C1_SDA_GPIO, SOFTI2C1_SDA_PIN,
      SOFTI2C1_SCL_GPIO, SOFTI2C1_SCL_PIN);

  /* Set default bitrate */
  i2c_setbitrate(&softi2c1, SOFTI2C1_CLOCK_SPEED);
}
#endif /* USE_SOFTI2C1 */


/*
 * I2C implementation
 */
// I2C Standard-mode timing
// Refer to NXP UM10204 chapter 6.
// All times in seconds
#define T_HD_STA_MIN (4.0e-6f)
#define T_LOW_MIN    (4.7e-6f)
#define T_HIGH_MIN   (4.0e-6f)
#define T_SU_STA_MIN (4.7e-6f)
#define T_HD_DAT_MIN (5.0e-6f)
#define T_SU_DAT_MIN (0.25e-6f)
#define T_R_MAX      (1000.0e-9f)
#define T_F_MAX      (300.0e-9f)
#define T_SU_STO_MIN (4.0e-6f)
#define T_BUF_MIN    (4.7e-6f)
#define T_VD_DAT_MAX (3.45e-6f)
#define T_VD_ACK_MAX (3.45e-6f)

// Bit read/write functions
// Should be called continuously from event function.
// Return true upon completion.
// Unless otherwise mentioned, the bit functions start at SCL <= low and end at
// the earliest moment that SCL can be set low again. This also takes the
// configured bit rate into account!

// write_start:
// starts at SDA <= low
// ends at   SCL allowed low
static bool softi2c_write_start(struct softi2c_device *d) {
  float bit_time = time.now - d->bit_state_time;
  switch (d->bit_state) {
    case 0:
      // Start of bit
      softi2c_gpio_drive_low(d->sda_port, d->sda_pin);
      d->bit_state_time = time.now;
      d->bit_state++;
      return false;
    case 1:
      if (bit_time < T_F_MAX + T_HD_STA_MIN) return false;
      // After SDA fall time and start condition hold time
      d->bit_state = 0;
      return true;
    default: return false;
  }
}

// write_bit:
// starts at SCL <= low
// ends at   SCL allowed low
// Note: write_bit may also be used to write the ACK bit (T_VD_ACK == T_VD_DAT)
static bool softi2c_write_bit(struct softi2c_device *d, bool bit) {
  float bit_time = time.now - d->bit_state_time;
  float bit_total_time = time.now - d->bit_start_time;
  switch (d->bit_state) {
    case 0:
      // Start of bit
      d->bit_start_time = time.now;
      softi2c_gpio_drive_low(d->scl_port, d->scl_pin);
      d->bit_state_time = time.now;
      d->bit_state++;
      return false;
    case 1:
      if (bit_time < T_F_MAX + T_HD_DAT_MIN) return false;
      // After SCL fall time and data hold time
      if (bit) {
        softi2c_gpio_highz(d->sda_port, d->sda_pin);
      } else {
        softi2c_gpio_drive_low(d->sda_port, d->sda_pin);
      }
      d->bit_state_time = time.now;
      d->bit_state++;
      return false;
    case 2:
      if (bit_time < T_R_MAX + T_SU_DAT_MIN) return false;
      // After SDA rise(/fall) and data set-up time
      softi2c_gpio_highz(d->scl_port, d->scl_pin);
      d->bit_state_time = time.now;
      d->bit_state++;
      return false;
    case 3:
      if (bit_time < T_R_MAX) return false;
      if (!gpio_get(d->scl_port, d->scl_pin)) return false;
      // After SCL rise time and confirmed high (clock stretching)
      d->bit_state_time = time.now;
      d->bit_state++;
      return false;
    case 4:
      if (bit_time < T_HIGH_MIN || bit_total_time < d->t_scl) return false;
      // After T_HIGH
      d->bit_state = 0;
      return true;
    default: return false;
  }
}

// read_bit:
// starts at SCL <= low
// ends at   SCL allowed low
// Note: read_bit may also be used to read the ACK bit (T_VD_ACK == T_VD_DAT)
static bool softi2c_read_bit(struct softi2c_device *d, bool *bit) {
  float bit_time = time.now - d->bit_state_time;
  float bit_total_time = time.now - d->bit_start_time;
  switch (d->bit_state) {
    case 0:
      // Start of bit
      d->bit_start_time = time.now;
      softi2c_gpio_drive_low(d->scl_port, d->scl_pin);
      d->bit_state_time = time.now;
      d->bit_state++;
      return false;
    case 1:
      if (bit_time < T_F_MAX + T_HD_DAT_MIN) return false;
      // After SCL fall time and data hold time
      softi2c_gpio_highz(d->sda_port, d->sda_pin);  // SDA may be driven low by slave.
      // d->bit_step_time = time.now;  // Read timing based on SCL <= low.
      d->bit_state++;
      return false;
    case 2:
      if (bit_time < T_F_MAX + T_LOW_MIN) return false;
      // After SCL(!) fall time and minimum low time
      softi2c_gpio_highz(d->scl_port, d->scl_pin);
      d->bit_state_time = time.now;
      d->bit_state++;
      return false;
    case 3:
      if (bit_time < T_R_MAX) return false;
      if (!gpio_get(d->scl_port, d->scl_pin)) return false;
      // After SCL rise time and confirmed high (clock stretching)
      d->bit_state_time = time.now;
      d->bit_state++;
      return false;
    case 4:
      if (bit_time < T_HIGH_MIN || bit_total_time < d->t_scl) return false;
      // After SCL minimum high time
      *bit = softi2c_gpio_read(d->sda_port, d->sda_pin);
      d->bit_state = 0;
      return true;
    default: return false;
  }
}


// write_restart:
// starts at SCL <= low
// ends at   SCL allowed low
static bool softi2c_write_restart(struct softi2c_device *d) {
  float bit_time = time.now - d->bit_state_time;
  float bit_total_time = time.now - d->bit_start_time;
  switch (d->bit_state) {
    case 0:
      // Start of bit
      d->bit_start_time = time.now;
      softi2c_gpio_drive_low(d->scl_port, d->scl_pin);
      d->bit_state_time = time.now;
      d->bit_state++;
      return false;
    case 1:
      if (bit_time < T_F_MAX + T_HD_DAT_MIN) return false;
      // After SCL fall time and data hold time
      softi2c_gpio_highz(d->sda_port, d->sda_pin);
      d->bit_state_time = time.now;
      d->bit_state++;
      return false;
    case 2:
      if (bit_time < T_R_MAX + T_SU_DAT_MIN) return false;
      // After SDA rise time and data set-up time
      softi2c_gpio_highz(d->scl_port, d->scl_pin);
      d->bit_state_time = time.now;
      d->bit_state++;
      return false;
    case 3:
      if (bit_time < T_R_MAX) return false;
      if (!gpio_get(d->scl_port, d->scl_pin)) return false;
      // After SCL rise time and confirmed high (clock stretching)
      d->bit_state_time = time.now;
      d->bit_state++;
      return false;
    case 4:
      if (bit_time < T_SU_STA_MIN) return false;
      // After restart set-up time
      softi2c_gpio_drive_low(d->sda_port, d->sda_pin);
      d->bit_state_time = time.now;
      d->bit_state++;
      return false;
    case 5:
      if (bit_time < T_F_MAX + T_HD_STA_MIN || bit_total_time < d->t_scl) return false;
      // After SDA fall time and restart hold time
      d->bit_state = 0;
      return true;
    default: return false;
  }
}

// write_stop:
// starts at SCL <= low
// ends at   SCL allowed low
static bool softi2c_write_stop(struct softi2c_device *d) {
  float bit_time = time.now - d->bit_state_time;
    switch (d->bit_state) {
      case 0:
        // Start of bit
        softi2c_gpio_drive_low(d->scl_port, d->scl_pin);
        d->bit_state_time = time.now;
        d->bit_state++;
        return false;
      case 1:
        if (bit_time < T_F_MAX + T_HD_DAT_MIN) return false;
        // After SCL fall time and data hold time
        softi2c_gpio_drive_low(d->sda_port, d->sda_pin);
        d->bit_state_time = time.now;
        d->bit_state++;
        return false;
      case 2:
        if (bit_time < T_F_MAX + T_SU_DAT_MIN) return false;
        // After SDA fall time and data set-up time
        softi2c_gpio_highz(d->scl_port, d->scl_pin);
        d->bit_state_time = time.now;
        d->bit_state++;
        return false;
      case 3:
        if (bit_time < T_R_MAX) return false;
        if (!gpio_get(d->scl_port, d->scl_pin)) return false;
        // After SCL rise time and confirmed high (clock stretching)
        d->bit_state_time = time.now;
        d->bit_state++;
        return false;
      case 4:
        if (bit_time < T_SU_STO_MIN) return false;
        // After stop set-up time
        softi2c_gpio_highz(d->sda_port, d->sda_pin);
        d->bit_state_time = time.now;
        d->bit_state++;
        return false;
      case 5:
        if (bit_time < T_R_MAX + T_BUF_MIN) return false;
        // After SDA rise time and bus free time
        d->bit_state = 0;
        return true;
      default: return false;
    }
}

// Byte read/write functions
// Should be called continuously from event function.
// Return true upon completion.

// write_byte:
// starts with SCL <= low for first bit (start bit is not included!)
// ends at SCL allowed low
static bool softi2c_write_byte(struct softi2c_device *d, uint8_t byte, bool *ack) {
  bool bit;
  switch (d->byte_state) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
      // Write bit
      bit = byte & (0x80 >> d->byte_state); // MSB first
      if (!softi2c_write_bit(d, bit)) return false;  // Write bit in progress
      d->byte_state++;
      return false;
    case 8:
      // Read ACK
      if (!softi2c_read_bit(d, ack)) return false;  // Read bit in progress
      d->byte_state = 0;
      return true;
    default: return false;
  }
}

// read_byte:
// starts with SCL <= low for first bit (start bit is not included!)
// ends at SCL allowed low
// Note: ack should be false when reading last byte (UM10204 3.1.10)
static bool softi2c_read_byte(struct softi2c_device *d, uint8_t *byte, bool ack) {
  bool bit;
  switch (d->byte_state) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
      // Read bit
      if (!softi2c_read_bit(d, &bit)) return false;  // Read bit in progress
      if (bit) {
        *byte |= (0x80 >> d->byte_state);  // MSB first
      } else {
        *byte &= ~(0x80 >> d->byte_state);  // MSB first
      }
      d->byte_state++;
      return false;
    case 8:
      // Write ACK
      if (!softi2c_write_bit(d, ack)) return false;  // Write bit in progress
      d->byte_state = 0;
      return true;
    default: return false;
  }
}

// Transaction handling
// Should be called continously from event function.
// Returns true upon completion.
static bool softi2c_process_transaction(struct softi2c_device *d, struct i2c_transaction *t) {
  uint8_t byte;
  bool ack;
  switch (d->periph->status) {
    case I2CIdle:
      // Start of transaction
      t->status = I2CTransRunning;
      d->periph->status = I2CStartRequested;
      d->periph->idx_buf = 0;
      return false;

    case I2CStartRequested:
      // Send start bit
      if (!softi2c_write_start(d)) return false;
      // Start bit sent
      if (t->type == I2CTransRx) {
        d->periph->status = I2CAddrRdSent;
      } else {
        d->periph->status = I2CAddrWrSent;
      }
      return false;

    case I2CAddrWrSent:
      // Send write address
      if (!softi2c_write_byte(d, t->slave_addr, &ack)) return false;
      // Write address sent
      if (!ack) {
        d->periph->errors->ack_fail_cnt++;
        t->status = I2CTransFailed;
        d->periph->status = I2CStopRequested;
        return false;
      }
      d->periph->status = I2CSendingByte;
      return false;

    case I2CSendingByte:
      // Check remaining bytes
      if (d->periph->idx_buf >= t->len_w) {
        d->periph->idx_buf = 0;
        if (t->type == I2CTransTxRx) {
          d->periph->status = I2CRestartRequested;
        } else {
          t->status = I2CTransSuccess;
          d->periph->status = I2CStopRequested;
        }
        return false;
      }
      // Send byte
      if (!softi2c_write_byte(d, t->buf[d->periph->idx_buf], &ack)) return false;
      // Byte sent
      if (!ack) {
        d->periph->errors->ack_fail_cnt++;
        t->status = I2CTransFailed;
        d->periph->status = I2CStopRequested;
        return true;
      }
      d->periph->idx_buf++;
      return false;

    case I2CRestartRequested:
      // Send restart bit
      if (!softi2c_write_restart(d)) return false;
      // Restart bit sent
      d->periph->status = I2CAddrRdSent;
      return false;

    case I2CAddrRdSent:
      // Send read address
      byte = t->slave_addr | 0x01;
      if (!softi2c_write_byte(d, byte, &ack)) return false;
      // Read address sent
      if (!ack) {
        d->periph->errors->ack_fail_cnt++;
        t->status = I2CTransFailed;
        d->periph->status = I2CStopRequested;
        return true;
      }
      d->periph->status = I2CReadingByte;
      return false;

    case I2CReadingByte:
      // Check remaining bytes
      if (d->periph->idx_buf >= t->len_r - 1) {
        d->periph->status = I2CReadingLastByte;
        return false;
      }
      // Read byte
      if (!softi2c_read_byte(d, (uint8_t *) &(t->buf[d->periph->idx_buf]), true)) return false;
      // Byte read
      d->periph->idx_buf++;
      return false;

    case I2CReadingLastByte:
      // Check remaining bytes
      if (d->periph->idx_buf >= t->len_r) {
        t->status = I2CTransSuccess;
        d->periph->idx_buf = 0;
        d->periph->status = I2CStopRequested;
        return false;
      }
      // Read last byte (no ACK!)
      if (!softi2c_read_byte(d, (uint8_t *) &(t->buf[d->periph->idx_buf]), false)) return false;
      // Byte read
      d->periph->idx_buf++;
      return false;

    case I2CStopRequested:
      // Send stop bit
      if (!softi2c_write_stop(d)) return false;
      // Stop bit sent
      d->periph->status = I2CIdle;
      return true;

    default:
      // Should never happen, something went wrong
      t->status = I2CTransFailed;
      d->periph->status = I2CIdle;
      return true;
  }
}

// Per-device event function
static void softi2c_device_event(struct softi2c_device *d) __attribute__((unused));

static void softi2c_device_event(struct softi2c_device *d) {
  struct i2c_periph *p = d->periph;
  if (p->trans_insert_idx != p->trans_extract_idx) {
    // Transaction(s) in queue
    struct i2c_transaction *t = p->trans[p->trans_extract_idx];
    if (softi2c_process_transaction(d, t)) {
      // Transaction finished
      p->trans_extract_idx = (p->trans_extract_idx + 1) % I2C_TRANSACTION_QUEUE_LEN;
    }
  }
}


// Timing function
static void update_highres_clock(void) {
  float sys_time_now = get_sys_time_float();
  // Initialize clock if req'd
  if (time.last_sys_time == 0.0) {
    time.last_sys_time = sys_time_now;
    time.time_per_event = 5e-6; // Initial guess
  }
  // Re-estimate clock rate
  if (sys_time_now > time.last_sys_time + 10e-3) {
    time.time_per_event = (sys_time_now - time.last_sys_time) / (float)time.event_cnt;
    time.last_sys_time = sys_time_now;
    time.event_cnt = 0;
  }
  // Run clock
  time.now += time.time_per_event;
  time.event_cnt++;
}


/*
 * Paparazzi functions
 */
void softi2c_event(void) {
  update_highres_clock();
  for (int i = 0; i < SOFTI2C_EVENT_MULTIPLIER; ++i) {
#if USE_SOFTI2C0
  softi2c_device_event(&softi2c0_device);
#endif
#if USE_SOFTI2C1
  softi2c_device_event(&softi2c1_device);
#endif
  }
}

static bool softi2c_idle(struct i2c_periph *p) {
  return (p->status == I2CIdle) && (p->trans_insert_idx == p->trans_extract_idx);
}

static bool softi2c_submit(struct i2c_periph *p, struct i2c_transaction *t) {
  uint8_t next_idx = (p->trans_insert_idx + 1) % I2C_TRANSACTION_QUEUE_LEN;
  if (next_idx == p->trans_extract_idx) {
    // queue full
    p->errors->queue_full_cnt++;
    t->status = I2CTransFailed;
    return false;
  }
  t->status = I2CTransPending;
  /* put transaction in queue */
  p->trans[p->trans_insert_idx] = t;
  p->trans_insert_idx = next_idx;
  return true;
}

static void softi2c_setbitrate(struct i2c_periph *p, int bitrate) {
  struct softi2c_device *d = (struct softi2c_device *) p->reg_addr;
  d->t_scl = 1.0 / (float) bitrate;
}
