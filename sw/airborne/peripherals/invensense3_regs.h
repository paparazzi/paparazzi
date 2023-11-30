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
 * @file peripherals/invensense3_regs.h
 *
 * Register and address definitions for the Invensense V3 from ardupilot.
 */

#ifndef INVENSENSE3_REGS_H
#define INVENSENSE3_REGS_H

#define INV3_BANK0 0x00U
#define INV3_BANK1 0x01U
#define INV3_BANK2 0x02U
#define INV3_BANK3 0x03U


#define INV3REG(b, r)      ((((uint16_t)b) << 8)|(r))
#define INV3_READ_FLAG     0x80

//Register Map
#define INV3REG_DEVICE_CONFIG         INV3REG(INV3_BANK0,0x11U)
#       define BIT_DEVICE_CONFIG_SOFT_RESET_CONFIG          0x01
#       define BIT_DEVICE_CONFIG_SPI_MODE                   0x10
#define INV3REG_FIFO_CONFIG           INV3REG(INV3_BANK0,0x16U)
#       define FIFO_CONFIG_MODE_BYPASS                      0x00
#       define FIFO_CONFIG_MODE_STREAM_TO_FIFO              0x01
#       define FIFO_CONFIG_MODE_STOP_ON_FULL                0x02
#       define FIFO_CONFIG_MODE_SHIFT                       0x06
#define INV3REG_TEMP_DATA1            INV3REG(INV3_BANK0,0x1DU)
#define INV3REG_ACCEL_DATA_X1         INV3REG(INV3_BANK0,0x1FU)
#define INV3REG_INT_STATUS            INV3REG(INV3_BANK0,0x2DU)
#define INV3REG_FIFO_COUNTH           INV3REG(INV3_BANK0,0x2EU)
#define INV3REG_FIFO_COUNTL           INV3REG(INV3_BANK0,0x2FU)
#define INV3REG_FIFO_DATA             INV3REG(INV3_BANK0,0x30U)
#define INV3REG_SIGNAL_PATH_RESET     INV3REG(INV3_BANK0,0x4BU)
#       define BIT_SIGNAL_PATH_RESET_FIFO_FLUSH             0x02
#       define BIT_SIGNAL_PATH_RESET_TMST_STROBE            0x04
#       define BIT_SIGNAL_PATH_RESET_ABORT_AND_RESET        0x08
#       define BIT_SIGNAL_PATH_RESET_DMP_MEM_RESET_EN       0x20
#       define BIT_SIGNAL_PATH_RESET_DMP_INIT_EN            0x40
#define INV3REG_INTF_CONFIG0          INV3REG(INV3_BANK0,0x4CU)
#       define UI_SIFS_CFG_SPI_DIS                          0x02
#       define UI_SIFS_CFG_I2C_DIS                          0x03
#       define UI_SIFS_CFG_SHIFT                            0x00
#       define SENSOR_DATA_BIG_ENDIAN                       0x10
#       define FIFO_COUNT_BIG_ENDIAN                        0x20
#       define FIFO_COUNT_REC                               0x40
#       define FIFO_HOLD_LAST_DATA_EN                       0x80
#define INV3REG_INTF_CONFIG1          INV3REG(INV3_BANK0,0x4DU)
#define INV3REG_PWR_MGMT0             INV3REG(INV3_BANK0,0x4EU)
#       define ACCEL_MODE_OFF                               0x00
#       define ACCEL_MODE_LN                                0x03
#       define ACCEL_MODE_SHIFT                             0x00
#       define GYRO_MODE_OFF                                0x00
#       define GYRO_MODE_LN                                 0x03
#       define GYRO_MODE_SHIFT                              0x02
#       define BIT_PWR_MGMT_IDLE                            0x08
#       define BIT_PWM_MGMT_TEMP_DIS                        0x10
#define INV3REG_GYRO_CONFIG0          INV3REG(INV3_BANK0,0x4FU)
#       define GYRO_ODR_32KHZ                               0x01
#       define GYRO_ODR_16KHZ                               0x02
#       define GYRO_ODR_8KHZ                                0x03
#       define GYRO_ODR_4KHZ                                0x04
#       define GYRO_ODR_2KHZ                                0x05
#       define GYRO_ODR_1KHZ                                0x06
#       define GYRO_ODR_200HZ                               0x07
#       define GYRO_ODR_100HZ                               0x08
#       define GYRO_ODR_50HZ                                0x09
#       define GYRO_ODR_25HZ                                0x0A
#       define GYRO_ODR_12_5HZ                              0x0B
#       define GYRO_ODR_500HZ                               0x0F
#       define GYRO_ODR_SHIFT                               0x00
#       define GYRO_FS_SEL_2000DPS                          0x00
#       define GYRO_FS_SEL_1000DPS                          0x01
#       define GYRO_FS_SEL_500DPS                           0x02
#       define GYRO_FS_SEL_250DPS                           0x03
#       define GYRO_FS_SEL_125DPS                           0x04
#       define GYRO_FS_SEL_62_5DPS                          0x05
#       define GYRO_FS_SEL_31_25DPS                         0x06
#       define GYRO_FS_SEL_15_625DPS                        0x07
#       define GYRO_FS_SEL_SHIFT                            0x05
#define INV3REG_ACCEL_CONFIG0         INV3REG(INV3_BANK0,0x50U)
#       define ACCEL_ODR_32KHZ                              0x01
#       define ACCEL_ODR_16KHZ                              0x02
#       define ACCEL_ODR_8KHZ                               0x03
#       define ACCEL_ODR_4KHZ                               0x04
#       define ACCEL_ODR_2KHZ                               0x05
#       define ACCEL_ODR_1KHZ                               0x06
#       define ACCEL_ODR_200HZ                              0x07
#       define ACCEL_ODR_100HZ                              0x08
#       define ACCEL_ODR_50HZ                               0x09
#       define ACCEL_ODR_25HZ                               0x0A
#       define ACCEL_ODR_12_5HZ                             0x0B
#       define ACCEL_ODR_6_25HZ                             0x0C
#       define ACCEL_ODR_3_125HZ                            0x0D
#       define ACCEL_ODR_1_5625HZ                           0x0E
#       define ACCEL_ODR_500HZ                              0x0F
#       define ACCEL_ODR_SHIFT                              0x00
#       define ACCEL_FS_SEL_16G                             0x00
#       define ACCEL_FS_SEL_8G                              0x01
#       define ACCEL_FS_SEL_4G                              0x02
#       define ACCEL_FS_SEL_2G                              0x03
#       define ACCEL_FS_SEL_SHIFT                           0x05
#define INV3REG_GYRO_CONFIG1          INV3REG(INV3_BANK0,0x51U)
#define INV3REG_GYRO_ACCEL_CONFIG0    INV3REG(INV3_BANK0,0x52U)
#define INV3REG_ACCEL_CONFIG1         INV3REG(INV3_BANK0,0x53U)
#define INV3REG_TMST_CONFIG           INV3REG(INV3_BANK0,0x54U)
#       define BIT_TMST_CONFIG_TMST_EN                      0x01
#define INV3REG_FIFO_CONFIG1          INV3REG(INV3_BANK0,0x5FU)
#       define BIT_FIFO_CONFIG1_ACCEL_EN                    0x01
#       define BIT_FIFO_CONFIG1_GYRO_EN                     0x02
#       define BIT_FIFO_CONFIG1_TEMP_EN                     0x04
#       define BIT_FIFO_CONFIG1_TMST_FSYNC_EN               0x08
#       define BIT_FIFO_CONFIG1_HIRES_EN                    0x10
#       define BIT_FIFO_CONFIG1_WM_GT_TH                    0x20
#       define BIT_FIFO_CONFIG1_RESUME_PARTIAL_RD           0x40
#define INV3REG_FIFO_CONFIG2          INV3REG(INV3_BANK0,0x60U)
#define INV3REG_FIFO_CONFIG3          INV3REG(INV3_BANK0,0x61U)
#define INV3REG_INT_SOURCE0           INV3REG(INV3_BANK0,0x65U)
#define INV3REG_INT_SOURCE3           INV3REG(INV3_BANK0,0x68U)
#       define BIT_FIFO_FULL_INT_EN                        0x02
#       define BIT_FIFO_THS_INT_EN                         0x04
#       define BIT_UI_DRDY_INT_EN                          0x08
#define INV3REG_INT_CONFIG1           INV3REG(INV3_BANK0,0x64U)
#       define BIT_INT_ASYNC_RESET                          0x10
#define INV3REG_WHO_AM_I              INV3REG(INV3_BANK0,0x75U)
#define INV3REG_GYRO_CONFIG_STATIC2   INV3REG(INV3_BANK1,0x0BU)
#       define BIT_GYRO_NF_DIS                              0x01
#       define BIT_GYRO_AAF_DIS                             0x02
#define INV3REG_GYRO_CONFIG_STATIC3   INV3REG(INV3_BANK1,0x0CU)
#       define GYRO_AAF_DELT_SHIFT                          0x00
#define INV3REG_GYRO_CONFIG_STATIC4   INV3REG(INV3_BANK1,0x0DU)
#       define GYRO_AAF_DELTSQR_LOW_SHIFT                   0x00 //[0:7]
#define INV3REG_GYRO_CONFIG_STATIC5   INV3REG(INV3_BANK1,0x0EU)
#       define GYRO_AAF_DELTSQR_HIGH_SHIFT                  0x00 //[11:8]
#       define GYRO_AAF_BITSHIFT_SHIFT                      0x04
#define INV3REG_GYRO_CONFIG_STATIC6   INV3REG(INV3_BANK1,0x0FU)
#       define GYRO_X_NF_COSWZ_LOW_SHIFT                    0x00 //[0:7]
#define INV3REG_GYRO_CONFIG_STATIC7   INV3REG(INV3_BANK1,0x10U)
#       define GYRO_Y_NF_COSWZ_LOW_SHIFT                    0x00 //[0:7]
#define INV3REG_GYRO_CONFIG_STATIC8   INV3REG(INV3_BANK1,0x11U)
#       define GYRO_Z_NF_COSWZ_LOW_SHIFT                    0x00 //[0:7]
#define INV3REG_GYRO_CONFIG_STATIC9   INV3REG(INV3_BANK1,0x12U)
#       define GYRO_X_NF_COSWZ_HIGH_SHIFT                   0x00 //[8]
#       define GYRO_Y_NF_COSWZ_HIGH_SHIFT                   0x01 //[8]
#       define GYRO_Z_NF_COSWZ_HIGH_SHIFT                   0x02 //[8]
#       define GYRO_X_NF_COSWZ_SEL_SHIFT                    0x03 //[0]
#       define GYRO_Y_NF_COSWZ_SEL_SHIFT                    0x04 //[0]
#       define GYRO_Z_NF_COSWZ_SEL_SHIFT                    0x05 //[0]
#define INV3REG_GYRO_CONFIG_STATIC10  INV3REG(INV3_BANK1,0x13U)
#       define GYRO_NF_BW_SEL_SHIFT                         0x04
#define INV3REG_ACCEL_CONFIG_STATIC2  INV3REG(INV3_BANK2,0x03U)
#       define ACCEL_AAF_DIS                                0x01
#       define ACCEL_AAF_DELT_SHIFT                         0x01
#define INV3REG_ACCEL_CONFIG_STATIC3  INV3REG(INV3_BANK2,0x04U)
#       define ACCEL_AAF_DELTSQR_LOW_SHIFT                  0x00 //[0:7]
#define INV3REG_ACCEL_CONFIG_STATIC4  INV3REG(INV3_BANK2,0x05U)
#       define ACCEL_AAF_DELTSQR_HIGH_SHIFT                 0x00 //[11:8]
#       define ACCEL_AAF_BITSHIFT_SHIFT                     0x04

#define INV3REG_BANK_SEL              0x76

// WHOAMI values
#define INV3_WHOAMI_ICM40605      0x33
#define INV3_WHOAMI_ICM40609      0x3b
#define INV3_WHOAMI_ICM42605      0x42
#define INV3_WHOAMI_ICM42688      0x47
#define INV3_WHOAMI_IIM42652      0x6f
#define INV3_WHOAMI_ICM42670      0x67

#endif /* INVENSENSE3_REGS_H */
