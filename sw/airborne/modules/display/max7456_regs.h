/*
 * Copyright (C) 2013 Chris
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
 * @file modules/display/max7456_regs.h
 * Maxim MAX7456 single-channel monochrome on-screen display driver.
 *
 * Registers definition
 */

#ifndef MAX7456_REGS_H
#define MAX7456_REGS_H

//OSD REGISTER ADDRESSES
#define OSD_VM0_REG      0x00
#define OSD_VM1_REG      0x01
#define OSD_DMM_REG      0x04
#define OSD_DMAH_REG     0x05
#define OSD_DMAL_REG     0x06
#define OSD_DMDI_REG     0x07
#define OSD_OSDBL_REG    0x6C
#define OSD_OSDBL_REG_R  0xEC
#define OSD_STAT_REG     0xA0

//OSD BIT POSITIONS
#define OSD_VIDEO_MODE_PAL        (1<<6)          // Default = NTSC
#define OSD_SYNC_INTERNAL         ((1<<5)|(1<<4)) // Default = AUTO
#define OSD_SYNC_EXTERNAL         ((1<<5)   // Default = AUTO
#define OSD_IMAGE_ENABLE          (1<<3)    // Default = OSD OFF
#define OSD_REFRESH_ON_NEXT_VSYNC (1<<2)    // Default = immediately refresh video
#define OSD_RESET                 (1<<1)    // VM0 reg, hardware set to 0 after reset
#define OSD_VOUT_DISABLE          (1<<0)    // default= VIDEO OUT ENABLED
#define OSD_8BIT_MODE             (1<<6)    // default= 16 BIT MODE
#define OSD_BLINK_CHAR            (1<<4)    // default= No BLINKING
#define OSD_INVERT_PIXELS         (1<<3)    // default= No INVERSION
#define OSD_CLEAR_DISPLAY_MEMORY  (1<<2)    // DMM reg, default = 0
#define OSD_AUTO_INCREMENT_MODE   (1<<0)    // default = NO AUTO INCREMENT

//OSD STATUS REGISTER BIT POSITIONS
#define OSD_UNUSED_FLAG           (1<<7)    // It is not used.
#define OSD_RESET_BUSY_FLAG       (1<<6)    // 0=power on reset completed, 1=busy with reset.
#define OSD_NVRAM_BUSY_FLAG       (1<<5)    // 0=memory available, 1=memory busy
#define OSD_VSYNC_ABSENT_FLAG     (1<<4)    // 0=vertical sync active, 1= no vsync
#define OSD_HSYNC_ABSENT_FLAG     (1<<3)    // 0=Horizontal sync active, 1= no Hsync
#define OSD_LOS_ABSENT_FLAG       (1<<2)    // 0=sync active, 1= no sync
#define OSD_NTSC_PRESENT_FLAG     (1<<1)    // 0=No NTSC signal detected, 1=NTSC signal detected.
#define OSD_PAL_PRESENT_FLAG      (1<<0)    // 0=No PAL signal detected, 1=PAL signal detected.

// MAX7456 VIDEO_MODE_0 register
#define VIDEO_MODE_0_WRITE              0x00
#define VIDEO_MODE_0_READ               0x80
#define VIDEO_MODE_0_40_PAL             0x40
#define VIDEO_MODE_0_20_NoAutoSync      0x20
#define VIDEO_MODE_0_10_SyncInt         0x10
#define VIDEO_MODE_0_08_EnOSD           0x08
#define VIDEO_MODE_0_04_UpdateVsync     0x04
#define VIDEO_MODE_0_02_Reset           0x02
#define VIDEO_MODE_0_01_EnVideo         0x01
// VIDEO MODE 0 bitmap
#define NTSC                            0x00
#define PAL                             0x40
#define AUTO_SYNC                       0x00
#define EXT_SYNC                        0x20
#define INT_SYNC                        0x30
#define OSD_EN                          0x08
#define VERT_SYNC_IMM                   0x00
#define VERT_SYNC_VSYNC                 0x04
#define SW_RESET                        0x02
#define BUF_EN                          0x00
#define BUF_DI                          0x01

// MAX7456 VIDEO_MODE_1 register
#define VIDEO_MODE_1_WRITE              0x01
#define VIDEO_MODE_1_READ               0x81

// MAX7456 DM_MODE register
#define DM_MODE_WRITE                   0x04
#define DM_MODE_READ                    0x84

// MAX7456 DM_ADDRH register
#define DM_ADDRH_WRITE                  0x05
#define DM_ADDRH_READ                   0x85

// MAX7456 DM_ADDRL register
#define DM_ADDRL_WRITE                  0x06
#define DM_ADDRL_READ                   0x87

// MAX7456 DM_CODE_IN register
#define DM_CODE_IN_WRITE                0x07
#define DM_CODE_IN_READ                 0x87

// MAX7456 DM_CODE_OUT register
#define DM_CODE_OUT_READ                0xB0

// MAX7456 FM_MODE register
#define FM_MODE_WRITE                   0x08
#define FM_MODE_READ                    0x88

// MAX7456 FM_ADDRH register
#define FM_ADDRH_WRITE                  0x09
#define FM_ADDRH_READ                   0x89

// MAX7456 FM_ADDRL register
#define FM_ADDRL_WRITE                  0x0A
#define FM_ADDRL_READ                   0x8A

// MAX7456 FM_DATA_IN register
#define FM_DATA_IN_WRITE                0x0B
#define FM_DATA_IN_READ                 0x8B

// MAX7456 FM_DATA_OUT register
#define FM_DATA_OUT_READ                0xC0

// MAX7456 requires clearing OSD Black Level register bit 0x10 after reset
#define OSDBL_WR                        0x6C
#define OSDBL_RD                        0xEC
#define OSDBL_10_DisableAutoBlackLevel  0x10

#endif //MAX7456_REGS_H
