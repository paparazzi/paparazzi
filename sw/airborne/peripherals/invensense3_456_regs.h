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
 * @file peripherals/invensense3_456_regs.h
 *
 * Register and address definitions for the Invensense V3 ICM456xy sensors from ardupilot.
 */

#ifndef INVENSENSE3_456_REGS_H
#define INVENSENSE3_456_REGS_H

#define INV3_456_READ_FLAG     0x80


#define ACCEL_ODR_SHIFT                              0x00
#define GYRO_ODR_SHIFT                              0x00
#define GYRO_FS_SEL_SHIFT                            0x04
#define ACCEL_FS_SEL_SHIFT                           0x04

// WHOAMI values
#define INV3_456_WHOAMI_ICM45686      0xE9  // REF: https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf

#define INV3REG_456_WHOAMI          0x72
#define INV3REG_456_PWR_MGMT0       0x10
#define INV3REG_456_INT1_STATUS0    0x19
#define INV3REG_456_ACCEL_CONFIG0   0x1B
#define INV3REG_456_GYRO_CONFIG0    0x1C
#define INV3REG_456_FIFO_CONFIG0    0x1D
#define INV3REG_456_FIFO_CONFIG2    0x20
#define INV3REG_456_FIFO_CONFIG3    0x21
#define INV3REG_456_FIFO_CONFIG4    0x22
#define INV3REG_456_RTC_CONFIG      0x26
#define INV3REG_456_FIFO_COUNTH     0x12
#define INV3REG_456_FIFO_COUNTL     0x13
#define INV3REG_456_FIFO_DATA       0x14
#define INV3REG_456_INTF_CONFIG0    0x2C
#define INV3REG_456_IOC_PAD_SCENARIO 0x2F
#define INV3REG_456_IOC_PAD_SCENARIO_AUX_OVRD 0x30
#define INV3REG_456_IOC_PAD_SCENARIO_OVRD 0x31
#define INV3REG_456_PWR_MGMT_AUX1   0x54
#define INV3REG_456_IREG_ADDRH      0x7C
#define INV3REG_456_IREG_ADDRL      0x7D
#define INV3REG_456_IREG_DATA       0x7E
#define INV3REG_456_REG_MISC2       0x7F
#define INV3REG_456_SREG_CTRL       0x63

#define INV3BANK_456_IMEM_SRAM_ADDR 0x0000
#define INV3BANK_456_IPREG_BAR_ADDR 0xA000
#define INV3BANK_456_IPREG_TOP1_ADDR 0xA200
#define INV3BANK_456_IPREG_SYS1_ADDR 0xA400
#define INV3BANK_456_IPREG_SYS2_ADDR 0xA500

#define INV_456_BASE_FIFO3_CONFIG_VALUE (1U<<3 | 1U<<2 | 1U<<1) // FIFO_HIRES_EN | FIFO_GYRO_EN | FIFO_ACCEL_EN
#define INV_456_FIFO_IF_EN (1U<<0) // INV_456_FIFO_IF_EN



#endif /* INVENSENSE3_456_REGS_H */
