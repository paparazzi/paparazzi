/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
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
 * @file boards/bebop/mt9v117.c
 * Initialization of MT9V117 chip and options to change settings
 */

#include "std.h"
#include "mt9v117.h"
#include "mt9v117_regs.h"
#include "peripherals/video_device.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>

#include "generated/airframe.h"
#ifdef BOARD_DISCO
#include "boards/disco.h"
#else
#include "boards/bebop.h"
#endif


/* Camera structure */
struct video_config_t bottom_camera = {
  .output_size = {
    .w = 240,
    .h = 240
  },
  .sensor_size = {
    .w = 320,
    .h = 240,
  },
  .crop = {
    .x = 40,
    .y = 0,
    .w = 240,
    .h = 240
  },
  .dev_name = "/dev/video0",
  .subdev_name = "/dev/v4l-subdev0",
  .format = V4L2_PIX_FMT_UYVY,
  .subdev_format = V4L2_MBUS_FMT_UYVY8_2X8,
  .buf_cnt = 5,
  .filters = 0,
  .cv_listener = NULL,
  .fps = MT9V117_TARGET_FPS,
  .camera_intrinsics = {
    .focal_x = MT9V117_FOCAL_X,
    .focal_y = MT9V117_FOCAL_Y,
    .center_x = MT9V117_CENTER_X,
    .center_y = MT9V117_CENTER_Y,
    .Dhane_k = MT9V117_DHANE_K
  }
};

struct mt9v117_t mt9v117 = {
  .i2c_periph = &i2c0
};

/* Patch lines */
//I2C_BUF_LEN must be higher then size of these patch lines
#define MT9V117_PATCH_LINE_NUM 13
static uint8_t patch_line1[] = {
  0xf0, 0x00, 0x72, 0xcf, 0xff, 0x00, 0x3e, 0xd0, 0x92, 0x00,
  0x71, 0xcf, 0xff, 0xff, 0xf2, 0x18, 0xb1, 0x10, 0x92, 0x05,
  0xb1, 0x11, 0x92, 0x04, 0xb1, 0x12, 0x70, 0xcf, 0xff, 0x00,
  0x30, 0xc0, 0x90, 0x00, 0x7f, 0xe0, 0xb1, 0x13, 0x70, 0xcf,
  0xff, 0xff, 0xe7, 0x1c, 0x88, 0x36, 0x09, 0x0f, 0x00, 0xb3
};

static uint8_t patch_line2[] = {
  0xf0, 0x30, 0x69, 0x13, 0xe1, 0x80, 0xd8, 0x08, 0x20, 0xca,
  0x03, 0x22, 0x71, 0xcf, 0xff, 0xff, 0xe5, 0x68, 0x91, 0x35,
  0x22, 0x0a, 0x1f, 0x80, 0xff, 0xff, 0xf2, 0x18, 0x29, 0x05,
  0x00, 0x3e, 0x12, 0x22, 0x11, 0x01, 0x21, 0x04, 0x0f, 0x81,
  0x00, 0x00, 0xff, 0xf0, 0x21, 0x8c, 0xf0, 0x10, 0x1a, 0x22
};

static uint8_t patch_line3[] = {
  0xf0, 0x60, 0x10, 0x44, 0x12, 0x20, 0x11, 0x02, 0xf7, 0x87,
  0x22, 0x4f, 0x03, 0x83, 0x1a, 0x20, 0x10, 0xc4, 0xf0, 0x09,
  0xba, 0xae, 0x7b, 0x50, 0x1a, 0x20, 0x10, 0x84, 0x21, 0x45,
  0x01, 0xc1, 0x1a, 0x22, 0x10, 0x44, 0x70, 0xcf, 0xff, 0x00,
  0x3e, 0xd0, 0xb0, 0x60, 0xb0, 0x25, 0x7e, 0xe0, 0x78, 0xe0
};

static uint8_t patch_line4[] = {
  0xf0, 0x90, 0x71, 0xcf, 0xff, 0xff, 0xf2, 0x18, 0x91, 0x12,
  0x72, 0xcf, 0xff, 0xff, 0xe7, 0x1c, 0x8a, 0x57, 0x20, 0x04,
  0x0f, 0x80, 0x00, 0x00, 0xff, 0xf0, 0xe2, 0x80, 0x20, 0xc5,
  0x01, 0x61, 0x20, 0xc5, 0x03, 0x22, 0xb1, 0x12, 0x71, 0xcf,
  0xff, 0x00, 0x3e, 0xd0, 0xb1, 0x04, 0x7e, 0xe0, 0x78, 0xe0
};

static uint8_t patch_line5[] = {
  0xf0, 0xc0, 0x70, 0xcf, 0xff, 0xff, 0xe7, 0x1c, 0x88, 0x57,
  0x71, 0xcf, 0xff, 0xff, 0xf2, 0x18, 0x91, 0x13, 0xea, 0x84,
  0xb8, 0xa9, 0x78, 0x10, 0xf0, 0x03, 0xb8, 0x89, 0xb8, 0x8c,
  0xb1, 0x13, 0x71, 0xcf, 0xff, 0x00, 0x30, 0xc0, 0xb1, 0x00,
  0x7e, 0xe0, 0xc0, 0xf1, 0x09, 0x1e, 0x03, 0xc0, 0xc1, 0xa1
};

static uint8_t patch_line6[] = {
  0xf0, 0xf0, 0x75, 0x08, 0x76, 0x28, 0x77, 0x48, 0xc2, 0x40,
  0xd8, 0x20, 0x71, 0xcf, 0x00, 0x03, 0x20, 0x67, 0xda, 0x02,
  0x08, 0xae, 0x03, 0xa0, 0x73, 0xc9, 0x0e, 0x25, 0x13, 0xc0,
  0x0b, 0x5e, 0x01, 0x60, 0xd8, 0x06, 0xff, 0xbc, 0x0c, 0xce,
  0x01, 0x00, 0xd8, 0x00, 0xb8, 0x9e, 0x0e, 0x5a, 0x03, 0x20
};

static uint8_t patch_line7[] = {
  0xf1, 0x20, 0xd9, 0x01, 0xd8, 0x00, 0xb8, 0x9e, 0x0e, 0xb6,
  0x03, 0x20, 0xd9, 0x01, 0x8d, 0x14, 0x08, 0x17, 0x01, 0x91,
  0x8d, 0x16, 0xe8, 0x07, 0x0b, 0x36, 0x01, 0x60, 0xd8, 0x07,
  0x0b, 0x52, 0x01, 0x60, 0xd8, 0x11, 0x8d, 0x14, 0xe0, 0x87,
  0xd8, 0x00, 0x20, 0xca, 0x02, 0x62, 0x00, 0xc9, 0x03, 0xe0
};

static uint8_t patch_line8[] = {
  0xf1, 0x50, 0xc0, 0xa1, 0x78, 0xe0, 0xc0, 0xf1, 0x08, 0xb2,
  0x03, 0xc0, 0x76, 0xcf, 0xff, 0xff, 0xe5, 0x40, 0x75, 0xcf,
  0xff, 0xff, 0xe5, 0x68, 0x95, 0x17, 0x96, 0x40, 0x77, 0xcf,
  0xff, 0xff, 0xe5, 0x42, 0x95, 0x38, 0x0a, 0x0d, 0x00, 0x01,
  0x97, 0x40, 0x0a, 0x11, 0x00, 0x40, 0x0b, 0x0a, 0x01, 0x00
};

static uint8_t patch_line9[] = {
  0xf1, 0x80, 0x95, 0x17, 0xb6, 0x00, 0x95, 0x18, 0xb7, 0x00,
  0x76, 0xcf, 0xff, 0xff, 0xe5, 0x44, 0x96, 0x20, 0x95, 0x15,
  0x08, 0x13, 0x00, 0x40, 0x0e, 0x1e, 0x01, 0x20, 0xd9, 0x00,
  0x95, 0x15, 0xb6, 0x00, 0xff, 0xa1, 0x75, 0xcf, 0xff, 0xff,
  0xe7, 0x1c, 0x77, 0xcf, 0xff, 0xff, 0xe5, 0x46, 0x97, 0x40
};

static uint8_t patch_line10[] = {
  0xf1, 0xb0, 0x8d, 0x16, 0x76, 0xcf, 0xff, 0xff, 0xe5, 0x48,
  0x8d, 0x37, 0x08, 0x0d, 0x00, 0x81, 0x96, 0x40, 0x09, 0x15,
  0x00, 0x80, 0x0f, 0xd6, 0x01, 0x00, 0x8d, 0x16, 0xb7, 0x00,
  0x8d, 0x17, 0xb6, 0x00, 0xff, 0xb0, 0xff, 0xbc, 0x00, 0x41,
  0x03, 0xc0, 0xc0, 0xf1, 0x0d, 0x9e, 0x01, 0x00, 0xe8, 0x04
};

static uint8_t patch_line11[] = {
  0xf1, 0xe0, 0xff, 0x88, 0xf0, 0x0a, 0x0d, 0x6a, 0x01, 0x00,
  0x0d, 0x8e, 0x01, 0x00, 0xe8, 0x7e, 0xff, 0x85, 0x0d, 0x72,
  0x01, 0x00, 0xff, 0x8c, 0xff, 0xa7, 0xff, 0xb2, 0xd8, 0x00,
  0x73, 0xcf, 0xff, 0xff, 0xf2, 0x40, 0x23, 0x15, 0x00, 0x01,
  0x81, 0x41, 0xe0, 0x02, 0x81, 0x20, 0x08, 0xf7, 0x81, 0x34
};

static uint8_t patch_line12[] = {
  0xf2, 0x10, 0xa1, 0x40, 0xd8, 0x00, 0xc0, 0xd1, 0x7e, 0xe0,
  0x53, 0x51, 0x30, 0x34, 0x20, 0x6f, 0x6e, 0x5f, 0x73, 0x74,
  0x61, 0x72, 0x74, 0x5f, 0x73, 0x74, 0x72, 0x65, 0x61, 0x6d,
  0x69, 0x6e, 0x67, 0x20, 0x25, 0x64, 0x20, 0x25, 0x64, 0x0a,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t patch_line13[] = {
  0xf2, 0x40, 0xff, 0xff, 0xe8, 0x28, 0xff, 0xff, 0xf0, 0xe8,
  0xff, 0xff, 0xe8, 0x08, 0xff, 0xff, 0xf1, 0x54
};

/* Patch lines structure */
struct mt9v117_patch_t {
  uint8_t *data;
  uint16_t len;
};

static const struct mt9v117_patch_t mt9v117_patch_lines[MT9V117_PATCH_LINE_NUM] = {
  {patch_line1, sizeof(patch_line1)},
  {patch_line2, sizeof(patch_line2)},
  {patch_line3, sizeof(patch_line3)},
  {patch_line4, sizeof(patch_line4)},
  {patch_line5, sizeof(patch_line5)},
  {patch_line6, sizeof(patch_line6)},
  {patch_line7, sizeof(patch_line7)},
  {patch_line8, sizeof(patch_line8)},
  {patch_line9, sizeof(patch_line9)},
  {patch_line10, sizeof(patch_line10)},
  {patch_line11, sizeof(patch_line11)},
  {patch_line12, sizeof(patch_line12)},
  {patch_line13, sizeof(patch_line13)}
};

/**
 * Write multiple bytes to a single register
 */
static void write_reg(struct mt9v117_t *mt, uint16_t addr, uint32_t val, uint16_t len)
{
  mt->i2c_trans.buf[0] = addr >> 8;
  mt->i2c_trans.buf[1] = addr & 0xFF;

  // Fix sigdness based on length
  if (len == 1) {
    mt->i2c_trans.buf[2] = val & 0xFF;
  } else if (len == 2) {
    mt->i2c_trans.buf[2] = (val >> 8) & 0xFF;
    mt->i2c_trans.buf[3] = val & 0xFF;
  } else if (len == 4) {
    mt->i2c_trans.buf[2] = (val >> 24) & 0xFF;
    mt->i2c_trans.buf[3] = (val >> 16) & 0xFF;
    mt->i2c_trans.buf[4] = (val >> 8) & 0xFF;
    mt->i2c_trans.buf[5] = val & 0xFF;
  } else {
    printf("[MT9V117] write_reg with incorrect length %d\r\n", len);
  }

  // Transmit the buffer
  i2c_blocking_transmit(mt->i2c_periph, &mt->i2c_trans, MT9V117_ADDRESS, len + 2);
}

/**
 * Read multiple bytes from a register
 */
static uint32_t read_reg(struct mt9v117_t *mt, uint16_t addr, uint16_t len)
{
  uint32_t ret = 0;
  mt->i2c_trans.buf[0] = addr >> 8;
  mt->i2c_trans.buf[1] = addr & 0xFF;

  // Transmit the buffer and receive back
  i2c_blocking_transceive(mt->i2c_periph, &mt->i2c_trans, MT9V117_ADDRESS, 2, len);

  /* Fix sigdness */
  for (uint8_t i = 0; i < len; i++) {
    ret |= mt->i2c_trans.buf[len - i - 1] << (8 * i);
  }
  return ret;
}

/* Write a byte to a var */
static void write_var(struct mt9v117_t *mt, uint16_t var, uint16_t offset, uint32_t val, uint16_t len)
{
  uint16_t addr = 0x8000 | (var << 10) | offset;
  write_reg(mt, addr, val, len);
}

/* Read a byte from a var */
static uint32_t read_var(struct mt9v117_t *mt, uint16_t var, uint16_t offset, uint16_t len)
{
  uint16_t addr = 0x8000 | (var << 10) | offset;
  return read_reg(mt, addr, len);
}

static inline void mt9v117_write_patch(struct mt9v117_t *mt)
{
  /* Errata item 2 */
  write_reg(mt, 0x301a, 0x10d0, 2);
  write_reg(mt, 0x31c0, 0x1404, 2);
  write_reg(mt, 0x3ed8, 0x879c, 2);
  write_reg(mt, 0x3042, 0x20e1, 2);
  write_reg(mt, 0x30d4, 0x8020, 2);
  write_reg(mt, 0x30c0, 0x0026, 2);
  write_reg(mt, 0x301a, 0x10d4, 2);

  /* Errata item 6 */
  write_var(mt, MT9V117_AE_TRACK_VAR, 0x0002, 0x00d3, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, 0x0078, 0x00a0, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, 0x0076, 0x0140, 2);

  /* Errata item 8 */
  write_var(mt, MT9V117_LOW_LIGHT_VAR, 0x0004, 0x00fc, 2);
  write_var(mt, MT9V117_LOW_LIGHT_VAR, 0x0038, 0x007f, 2);
  write_var(mt, MT9V117_LOW_LIGHT_VAR, 0x003a, 0x007f, 2);
  write_var(mt, MT9V117_LOW_LIGHT_VAR, 0x003c, 0x007f, 2);
  write_var(mt, MT9V117_LOW_LIGHT_VAR, 0x0004, 0x00f4, 2);

  /* Patch 0403; Critical; Sensor optimization */
  write_reg(mt, MT9V117_ACCESS_CTL_STAT, 0x0001, 2);
  write_reg(mt, MT9V117_PHYSICAL_ADDRESS_ACCESS, 0x7000, 2);

  /* Write patch */
  for (uint8_t i = 0; i < MT9V117_PATCH_LINE_NUM; ++i) {
    // Copy buffer
    for (uint8_t j = 0; j < mt9v117_patch_lines[i].len; ++j) {
      mt->i2c_trans.buf[j] = mt9v117_patch_lines[i].data[j];
    }

    // Transmit the buffer
    i2c_blocking_transmit(mt->i2c_periph, &mt->i2c_trans, mt->i2c_trans.slave_addr, mt9v117_patch_lines[i].len);
  }

  write_reg(mt, MT9V117_LOGICAL_ADDRESS_ACCESS, 0x0000, 2);
  write_var(mt, MT9V117_PATCHLDR_VAR, MT9V117_PATCHLDR_LOADER_ADDRESS_OFFSET, 0x05d8, 2);
  write_var(mt, MT9V117_PATCHLDR_VAR, MT9V117_PATCHLDR_PATCH_ID_OFFSET, 0x0403, 2);
  write_var(mt, MT9V117_PATCHLDR_VAR, MT9V117_PATCHLDR_FIRMWARE_ID_OFFSET, 0x00430104, 4);
  write_reg(mt, MT9V117_COMMAND, MT9V117_COMMAND_OK | MT9V117_COMMAND_APPLY_PATCH, 2);

  /* Wait for command OK */
  for (uint8_t retries = 100; retries > 0; retries--) {
    /* Wait 10ms */
    usleep(10000);

    /* Check the command */
    uint16_t cmd = read_reg(mt, MT9V117_COMMAND, 2);
    if ((cmd & MT9V117_COMMAND_APPLY_PATCH) == 0) {
      if ((cmd & MT9V117_COMMAND_OK) == 0) {
        printf("[MT9V117] Applying patch failed (No OK)\r\n");
      }
      return;
    }
  }

  printf("[MT9V117] Applying patch failed after 10 retries\r\n");
}

/* Configure the sensor */
static inline void mt9v117_config(struct mt9v117_t *mt)
{
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_SENSOR_CFG_X_ADDR_START_OFFSET, 16, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_SENSOR_CFG_X_ADDR_END_OFFSET, 663, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_SENSOR_CFG_Y_ADDR_START_OFFSET, 8, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_SENSOR_CFG_Y_ADDR_END_OFFSET,  501, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_SENSOR_CFG_CPIPE_LAST_ROW_OFFSET, 243, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_SENSOR_CFG_FRAME_LENGTH_LINES_OFFSET, 283, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_SENSOR_CONTROL_READ_MODE_OFFSET,
            MT9V117_CAM_SENSOR_CONTROL_Y_SKIP_EN, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_SENSOR_CFG_MAX_FDZONE_60_OFFSET, 1, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_SENSOR_CFG_TARGET_FDZONE_60_OFFSET, 1, 2);

  write_reg(mt, MT9V117_AE_TRACK_JUMP_DIVISOR, 0x03, 1);
  write_reg(mt, MT9V117_CAM_AET_SKIP_FRAMES, 0x02, 1);

  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_OUTPUT_WIDTH_OFFSET, 320, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_OUTPUT_HEIGHT_OFFSET, 240, 2);

  /* Set gain metric for 111.2 fps
   * The final fps depends on the input clock
   * (89.2fps on bebop) so a modification may be needed here */
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_LL_START_GAIN_METRIC_OFFSET, 0x03e8, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_LL_STOP_GAIN_METRIC_OFFSET, 0x1770, 2);

  /* set crop window */
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_CROP_WINDOW_XOFFSET_OFFSET, 0, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_CROP_WINDOW_YOFFSET_OFFSET, 0, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_CROP_WINDOW_WIDTH_OFFSET, 640, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_CROP_WINDOW_HEIGHT_OFFSET, 240, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_CROP_MODE_OFFSET, 3, 1);

  /* Enable auto-stats mode */
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_STAT_AWB_HG_WINDOW_XSTART_OFFSET, 0, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_STAT_AWB_HG_WINDOW_YSTART_OFFSET, 0, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_STAT_AWB_HG_WINDOW_XEND_OFFSET, 319, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_STAT_AWB_HG_WINDOW_YEND_OFFSET, 239, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_STAT_AE_INITIAL_WINDOW_XSTART_OFFSET, 2, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_STAT_AE_INITIAL_WINDOW_YSTART_OFFSET, 2, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_STAT_AE_INITIAL_WINDOW_XEND_OFFSET, 63, 2);
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_STAT_AE_INITIAL_WINDOW_YEND_OFFSET, 47, 2);
}

/**
 * Initialisation of the Aptina MT9V117 CMOS sensor
 * (1/6 inch VGA, bottom camera)
 */
void mt9v117_init(struct mt9v117_t *mt)
{
  /* bytes written to gpios/pwm */
  int wc = 0;
  /* Reset the device */
  int gpio129 = open("/sys/class/gpio/gpio129/value", O_WRONLY | O_CREAT | O_TRUNC, 0666);
  wc += write(gpio129, "0", 1);
  wc += write(gpio129, "1", 1);
  close(gpio129);

  if (wc != 2) {
    printf("[MT9V117] Couldn't write to GPIO 129\n");
  }

  /* Start PWM 9 (Which probably is the clock of the MT9V117) */
  //#define BEBOP_CAMV_PWM_FREQ 43333333
  int pwm9 = open("/sys/class/pwm/pwm_9/run", O_WRONLY | O_CREAT | O_TRUNC, 0666);
  wc = 0;
  wc += write(pwm9, "0", 1);
  wc += write(pwm9, "1", 1);
  close(pwm9);

  if (wc != 2) {
    printf("[MT9V117] Couldn't write to PWM\n");
  }

  //TODO: Make PWM and GPIO generic

  /* Wait 50ms */
  usleep(50000);

  /* Setup i2c transaction */
  mt->i2c_trans.status = I2CTransDone;

  /* See if the device is there and correct */
  uint16_t chip_id = read_reg(mt, MT9V117_CHIP_ID, 2);
  if (chip_id != MT9V117_CHIP_ID_RESP) {
    printf("[MT9V117] Didn't get correct response from CHIP_ID (expected: 0x%04X, got: 0x%04X)\r\n", MT9V117_CHIP_ID_RESP,
           chip_id);
    return;
  }

  /* Reset the device with software */
  write_reg(mt, MT9V117_RESET_MISC_CTRL, MT9V117_RESET_SOC_I2C, 2);
  write_reg(mt, MT9V117_RESET_MISC_CTRL, 0, 2);

  /* Wait 50ms */
  usleep(50000);

  /* Apply MT9V117 software patch */
  mt9v117_write_patch(mt);

  /* Set basic settings */
  write_var(mt, MT9V117_AWB_VAR, MT9V117_AWB_PIXEL_THRESHOLD_COUNT_OFFSET, 50000, 4);
  write_var(mt, MT9V117_AE_RULE_VAR, MT9V117_AE_RULE_ALGO_OFFSET, MT9V117_AE_RULE_ALGO_AVERAGE, 2);

  /* Set pixclk pad slew to 6 and data out pad slew to 1 */
  write_reg(mt, MT9V117_PAD_SLEW, read_reg(mt, MT9V117_PAD_SLEW, 2) | 0x0600 | 0x0001, 2);

  /* Configure the MT9V117 sensor */
  mt9v117_config(mt);

  /* Enable ITU656 */
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_OUTPUT_FORMAT_OFFSET,
            read_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_CAM_OUTPUT_FORMAT_OFFSET, 2) |
            MT9V117_CAM_OUTPUT_FORMAT_BT656_ENABLE, 2);

  /* Set autoexposure luma */
  write_var(mt, MT9V117_CAM_CTRL_VAR, MT9V117_AE_LUMA, MT9V117_TARGET_LUMA, 2);

  /* Apply the configuration */
  write_var(mt, MT9V117_SYSMGR_VAR, MT9V117_SYSMGR_NEXT_STATE_OFFSET, MT9V117_SYS_STATE_ENTER_CONFIG_CHANGE, 1);
  write_reg(mt, MT9V117_COMMAND, MT9V117_COMMAND_OK | MT9V117_COMMAND_SET_STATE, 2);

  /* Wait for command OK */
  for (uint8_t retries = 100; retries > 0; retries--) {
    /* Wait 10ms */
    usleep(10000);

    /* Check the command */
    uint16_t cmd = read_reg(mt, MT9V117_COMMAND, 2);
    if ((cmd & MT9V117_COMMAND_SET_STATE) == 0) {
      if ((cmd & MT9V117_COMMAND_OK) == 0) {
        printf("[MT9V117] Switching config failed (No OK)\r\n");
      }

      // Successfully configured!
      //printf("[MT9V117] Switching config OK\r\n");
      return;
    }
  }

  printf("[MT9V117] Could not switch to new config\r\n");
}
