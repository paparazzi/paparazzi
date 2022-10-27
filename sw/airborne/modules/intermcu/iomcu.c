/*
 * Copyright (C) 2022 Freek van tieen <freek.v.tienen@gmail.com>
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
 * @file modules/intermcu/iomcu.c
 * Driver to communicate with the ardupilot IO MCU
 */

#include "iomcu.h"
#include "mcu_periph/uart.h"
#include <string.h>

// 22 is enough for the rc_input page in one transfer
#define PKT_MAX_REGS 22
#define IOMCU_MAX_CHANNELS 16

struct __attribute__((packed)) IOPacket {
  uint8_t   count: 6;
  uint8_t   code: 2;
  uint8_t   crc;
  uint8_t   page;
  uint8_t   offset;
  uint16_t  regs[PKT_MAX_REGS];
};

/*
  values for pkt.code
 */
enum iocode {
  // read types
  CODE_READ = 0,
  CODE_WRITE = 1,

  // reply codes
  CODE_SUCCESS = 0,
  CODE_CORRUPT = 1,
  CODE_ERROR = 2
};

// IO pages
enum iopage {
  PAGE_CONFIG = 0,
  PAGE_STATUS = 1,
  PAGE_ACTUATORS = 2,
  PAGE_SERVOS = 3,
  PAGE_RAW_RCIN = 4,
  PAGE_RCIN = 5,
  PAGE_RAW_ADC = 6,
  PAGE_PWM_INFO = 7,
  PAGE_SETUP = 50,
  PAGE_DIRECT_PWM = 54,
  PAGE_FAILSAFE_PWM = 55,
  PAGE_MIXING = 200,
  PAGE_GPIO = 201,
};

// setup page registers
#define PAGE_REG_SETUP_FEATURES 0
#define P_SETUP_FEATURES_SBUS1_OUT  1
#define P_SETUP_FEATURES_SBUS2_OUT  2
#define P_SETUP_FEATURES_PWM_RSSI   4
#define P_SETUP_FEATURES_ADC_RSSI   8
#define P_SETUP_FEATURES_ONESHOT   16
#define P_SETUP_FEATURES_BRUSHED   32

#define PAGE_REG_SETUP_ARMING 1
#define P_SETUP_ARMING_IO_ARM_OK (1<<0)
#define P_SETUP_ARMING_FMU_ARMED (1<<1)
#define P_SETUP_ARMING_RC_HANDLING_DISABLED (1<<6)
#define P_SETUP_ARMING_SAFETY_DISABLE_ON  (1 << 11) // disable use of safety button for safety off->on
#define P_SETUP_ARMING_SAFETY_DISABLE_OFF (1 << 12) // disable use of safety button for safety on->off

#define PAGE_REG_SETUP_PWM_RATE_MASK 2
#define PAGE_REG_SETUP_DEFAULTRATE   3
#define PAGE_REG_SETUP_ALTRATE       4
#define PAGE_REG_SETUP_REBOOT_BL    10
#define PAGE_REG_SETUP_CRC      11
#define PAGE_REG_SETUP_SBUS_RATE    19
#define PAGE_REG_SETUP_IGNORE_SAFETY 20 /* bitmask of surfaces to ignore the safety status */
#define PAGE_REG_SETUP_HEATER_DUTY_CYCLE 21
#define PAGE_REG_SETUP_DSM_BIND     22
#define PAGE_REG_SETUP_RC_PROTOCOLS 23 // uses 2 slots, 23 and 24

// config page registers
#define PAGE_CONFIG_PROTOCOL_VERSION  0
#define PAGE_CONFIG_PROTOCOL_VERSION2 1
#define IOMCU_PROTOCOL_VERSION       4
#define IOMCU_PROTOCOL_VERSION2     10

// magic value for rebooting to bootloader
#define REBOOT_BL_MAGIC 14662

#define PAGE_REG_SETUP_FORCE_SAFETY_OFF 12
#define PAGE_REG_SETUP_FORCE_SAFETY_ON  14
#define FORCE_SAFETY_MAGIC 22027

struct page_config {
  uint16_t protocol_version;
  uint16_t protocol_version2;
};

struct page_reg_status {
  uint16_t freemem;
  uint32_t timestamp_ms;
  uint16_t vservo;
  uint16_t vrssi;
  uint32_t num_errors;
  uint32_t total_pkts;
  uint8_t flag_safety_off;
  uint8_t err_crc;
  uint8_t err_bad_opcode;
  uint8_t err_read;
  uint8_t err_write;
  uint8_t err_uart;
};

struct page_rc_input {
  uint8_t count;
  uint8_t flags_failsafe: 1;
  uint8_t flags_rc_ok: 1;
  uint8_t rc_protocol;
  uint16_t pwm[IOMCU_MAX_CHANNELS];
  int16_t rssi;
};

/*
  data for mixing on FMU failsafe
 */
struct page_mixing {
  uint16_t servo_min[IOMCU_MAX_CHANNELS];
  uint16_t servo_max[IOMCU_MAX_CHANNELS];
  uint16_t servo_trim[IOMCU_MAX_CHANNELS];
  uint8_t servo_function[IOMCU_MAX_CHANNELS];
  uint8_t servo_reversed[IOMCU_MAX_CHANNELS];

  // RC input arrays are in AETR order
  uint16_t rc_min[4];
  uint16_t rc_max[4];
  uint16_t rc_trim[4];
  uint8_t rc_reversed[IOMCU_MAX_CHANNELS];
  uint8_t rc_channel[4];

  // gain for elevon and vtail mixing, x1000
  uint16_t mixing_gain;

  // channel which when high forces mixer
  int8_t rc_chan_override;

  // is the throttle an angle input?
  uint8_t throttle_is_angle;

  // mask of channels which are pure manual in override
  uint16_t manual_rc_mask;

  // enabled needs to be 1 to enable mixing
  uint8_t enabled;

  uint8_t pad;
};

struct __attribute__((packed, aligned(2))) page_GPIO {
  uint8_t channel_mask;
  uint8_t output_mask;
};

static const uint8_t crc8_table[] = {
  0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
  0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
  0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
  0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
  0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
  0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
  0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
  0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
  0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
  0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
  0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
  0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
  0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
  0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
  0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
  0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
  0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
  0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
  0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
  0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
  0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
  0xfa, 0xfd, 0xf4, 0xf3
};

/*
  crc8 from trone driver by Luis Rodrigues
 */
static uint8_t crc_crc8(const uint8_t *p, uint8_t len)
{
  uint16_t crc = 0x0;

  while (len--) {
    const uint16_t i = (crc ^ *p++) & 0xFF;
    crc = (crc8_table[i] ^ (crc << 8)) & 0xFF;
  }

  return crc & 0xFF;
}

/**
 * @brief Write registers to the IO MCU
 *
 * @param page The page of the register
 * @param offset The offset on the page
 * @param count The amount of registers to write
 * @param regs The register data
 */
static void iomcu_write_registers(uint8_t page, uint8_t offset, uint8_t count, const uint16_t *regs)
{
  // When we want to write more registers do a recursive call of depth 1
  while (count > PKT_MAX_REGS) {
    iomcu_write_registers(page, offset, PKT_MAX_REGS, regs);
    offset += PKT_MAX_REGS;
    count -= PKT_MAX_REGS;
    regs += PKT_MAX_REGS;
  }

  // Create the packet
  struct IOPacket pkt;
  pkt.code = CODE_WRITE;
  pkt.count = count;
  pkt.page = page;
  pkt.offset = offset;
  pkt.crc = 0;
  memcpy(pkt.regs, regs, 2 * count);
  const uint8_t pkt_size = count * 2 + 4;
  pkt.crc = crc_crc8((const uint8_t *)&pkt, pkt_size);

  // Send the packet
  (INTERMCU_LINK).device.put_buffer(&(INTERMCU_LINK), 0, (uint8_t *)&pkt, pkt_size);
}

/**
 * @brief Write a single register to the IO MCU
 *
 * @param page The page of the register
 * @param offset The offset on the page
 * @param v The value to write to the single register
 */
static void iomcu_write_register(uint8_t page, uint8_t offset, uint16_t v)
{
  iomcu_write_registers(page, offset, 1, &v);
}

/**
 * @brief Set the IO MCU heater duty cycle
 *
 * @param duty_cycle The duty cycle to set the heater to [0-100%]
 */
void iomcu_set_heater_duty_cycle(uint8_t duty_cycle)
{
  iomcu_write_register(PAGE_SETUP, PAGE_REG_SETUP_HEATER_DUTY_CYCLE, duty_cycle);
}
