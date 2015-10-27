/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file boards/bebop/video.c
 * Initialization of the video specific parts of the Bebop
 */

#include "video.h"
#include "std.h"
#include "mt9f002.h"

#include <stdio.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <mcu_periph/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/types.h>

#include "boards/bebop.h"

struct video_config_t bottom_camera = {
  .w = 640,
  .h = 480,
  .dev_name = "/dev/video0",
  .subdev_name = NULL,
  .format = V4L2_PIX_FMT_UYVY,
  .buf_cnt = 60,
  .filters = 0
};

struct video_config_t front_camera = {
  .w = 1408,
  .h = 2112,
  .dev_name = "/dev/video1",
  .subdev_name = "/dev/v4l-subdev1",
  .format = V4L2_PIX_FMT_SGBRG10,
  .buf_cnt = 10,
  .filters = VIDEO_FILTER_DEBAYER
};

static bool_t write_reg(int fd, char *addr_val, uint8_t cnt)
{
  char resp[cnt - 2];
  uint8_t i;

  if (write(fd, addr_val, cnt) != cnt) {
    printf("Write failed!\n");
    return FALSE;
  }
  if (write(fd, addr_val, 2) != 2) {
    printf("Write2 failed!\n");
    return FALSE;
  }
  while (read(fd, resp, cnt - 2) != cnt - 2) { ; }
  for (i = 0; i < cnt - 2; i++) {
    if (resp[i] != addr_val[i + 2]) {
      printf("[video] Could not write register %X%X\n", addr_val[0], addr_val[1]);
      return write_reg(fd, addr_val, cnt);
    }
  }
  return TRUE;
}

static bool_t _write(int fd, char *data, uint8_t cnt)
{
  if (write(fd, data, cnt) != cnt) {
    printf("Failed!\n");
  }
  return TRUE;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"

/**
 * Initialisation of the Aptina MT9V117 CMOS sensor
 * (1/6 inch VGA, bottom camera)
 */
void mt9v117_init(void)
{
  struct timespec tim;
  char test[2];

  /* Start PWM 9 (Which probably is the clock of the MT9V117) */
  int pwm9 = open("/sys/class/pwm/pwm_9/run", O_WRONLY | O_CREAT | O_TRUNC, 0666);
  write(pwm9, "0", 1);
  write(pwm9, "1", 1);
  close(pwm9);

  tim.tv_sec = 0;
  tim.tv_nsec = 50000000;
  nanosleep(&tim, NULL);

  /* We open the i2c-0 (because else I needed to convert the strace) */
  int fd_i2c = open("/dev/i2c-0", O_RDWR);
  if (fd_i2c < 0) {
    printf("[MT9V117] Could not open i2c-0\n");
    return;
  }
  if (ioctl(fd_i2c, 0x703, 0x5d) < 0) {
    printf("[MT9V117] Could not change the i2c address to 0x5d\n");
    return;
  }

  /* First reset the device */
  write_reg(fd_i2c, "\x00\x1a\x00\x01", 4);
  write_reg(fd_i2c, "\x00\x1a\x00\x00", 4);
  tim.tv_sec = 0;
  tim.tv_nsec = 100000000;
  nanosleep(&tim, NULL);

  /* Now initialize the device */
  write_reg(fd_i2c, "\x30\x1a\x10\xd0", 4);
  write_reg(fd_i2c, "\x31\xc0\x14\x04", 4);
  write_reg(fd_i2c, "\x3e\xd8\x87\x9c", 4);
  write_reg(fd_i2c, "\x30\x42\x20\xe1", 4);
  write_reg(fd_i2c, "\x30\xd4\x80\x20", 4);
  write_reg(fd_i2c, "\x30\xc0\x00\x26", 4);
  write_reg(fd_i2c, "\x30\x1a\x10\xd4", 4);
  write_reg(fd_i2c, "\xa8\x02\x00\xd3", 4);
  write_reg(fd_i2c, "\xc8\x78\x00\xa0", 4);
  write_reg(fd_i2c, "\xc8\x76\x01\x40", 4);
  write_reg(fd_i2c, "\xbc\x04\x00\xfc", 4);
  write_reg(fd_i2c, "\xbc\x38\x00\x7f", 4);
  write_reg(fd_i2c, "\xbc\x3a\x00\x7f", 4);
  write_reg(fd_i2c, "\xbc\x3c\x00\x7f", 4);
  write_reg(fd_i2c, "\xbc\x04\x00\xf4", 4);

  _write(fd_i2c, "\x09\x82\x00\x01", 4);
  _write(fd_i2c, "\x09\x8a\x70\x00", 4);
  _write(fd_i2c,
         "\xf0\x00\x72\xcf\xff\x00\x3e\xd0\x92\x00\x71\xcf\xff\xff\xf2\x18\xb1\x10\x92\x05\xb1\x11\x92\x04\xb1\x12\x70\xcf\xff\x00\x30\xc0\x90\x00\x7f\xe0\xb1\x13\x70\xcf\xff\xff\xe7\x1c\x88\x36\x09\x0f\x00\xb3",
         50);
  _write(fd_i2c,
         "\xf0\x30\x69\x13\xe1\x80\xd8\x08\x20\xca\x03\x22\x71\xcf\xff\xff\xe5\x68\x91\x35\x22\x0a\x1f\x80\xff\xff\xf2\x18\x29\x05\x00\x3e\x12\x22\x11\x01\x21\x04\x0f\x81\x00\x00\xff\xf0\x21\x8c\xf0\x10\x1a\x22",
         50);
  _write(fd_i2c,
         "\xf0\x60\x10\x44\x12\x20\x11\x02\xf7\x87\x22\x4f\x03\x83\x1a\x20\x10\xc4\xf0\x09\xba\xae\x7b\x50\x1a\x20\x10\x84\x21\x45\x01\xc1\x1a\x22\x10\x44\x70\xcf\xff\x00\x3e\xd0\xb0\x60\xb0\x25\x7e\xe0\x78\xe0",
         50);
  _write(fd_i2c,
         "\xf0\x90\x71\xcf\xff\xff\xf2\x18\x91\x12\x72\xcf\xff\xff\xe7\x1c\x8a\x57\x20\x04\x0f\x80\x00\x00\xff\xf0\xe2\x80\x20\xc5\x01\x61\x20\xc5\x03\x22\xb1\x12\x71\xcf\xff\x00\x3e\xd0\xb1\x04\x7e\xe0\x78\xe0",
         50);
  _write(fd_i2c,
         "\xf0\xc0\x70\xcf\xff\xff\xe7\x1c\x88\x57\x71\xcf\xff\xff\xf2\x18\x91\x13\xea\x84\xb8\xa9\x78\x10\xf0\x03\xb8\x89\xb8\x8c\xb1\x13\x71\xcf\xff\x00\x30\xc0\xb1\x00\x7e\xe0\xc0\xf1\x09\x1e\x03\xc0\xc1\xa1",
         50);
  _write(fd_i2c,
         "\xf0\xf0\x75\x08\x76\x28\x77\x48\xc2\x40\xd8\x20\x71\xcf\x00\x03\x20\x67\xda\x02\x08\xae\x03\xa0\x73\xc9\x0e\x25\x13\xc0\x0b\x5e\x01\x60\xd8\x06\xff\xbc\x0c\xce\x01\x00\xd8\x00\xb8\x9e\x0e\x5a\x03\x20",
         50);
  _write(fd_i2c,
         "\xf1\x20\xd9\x01\xd8\x00\xb8\x9e\x0e\xb6\x03\x20\xd9\x01\x8d\x14\x08\x17\x01\x91\x8d\x16\xe8\x07\x0b\x36\x01\x60\xd8\x07\x0b\x52\x01\x60\xd8\x11\x8d\x14\xe0\x87\xd8\x00\x20\xca\x02\x62\x00\xc9\x03\xe0",
         50);
  _write(fd_i2c,
         "\xf1\x50\xc0\xa1\x78\xe0\xc0\xf1\x08\xb2\x03\xc0\x76\xcf\xff\xff\xe5\x40\x75\xcf\xff\xff\xe5\x68\x95\x17\x96\x40\x77\xcf\xff\xff\xe5\x42\x95\x38\x0a\x0d\x00\x01\x97\x40\x0a\x11\x00\x40\x0b\x0a\x01\x00",
         50);
  _write(fd_i2c,
         "\xf1\x80\x95\x17\xb6\x00\x95\x18\xb7\x00\x76\xcf\xff\xff\xe5\x44\x96\x20\x95\x15\x08\x13\x00\x40\x0e\x1e\x01\x20\xd9\x00\x95\x15\xb6\x00\xff\xa1\x75\xcf\xff\xff\xe7\x1c\x77\xcf\xff\xff\xe5\x46\x97\x40",
         50);
  _write(fd_i2c,
         "\xf1\xb0\x8d\x16\x76\xcf\xff\xff\xe5\x48\x8d\x37\x08\x0d\x00\x81\x96\x40\x09\x15\x00\x80\x0f\xd6\x01\x00\x8d\x16\xb7\x00\x8d\x17\xb6\x00\xff\xb0\xff\xbc\x00\x41\x03\xc0\xc0\xf1\x0d\x9e\x01\x00\xe8\x04",
         50);
  _write(fd_i2c,
         "\xf1\xe0\xff\x88\xf0\x0a\x0d\x6a\x01\x00\x0d\x8e\x01\x00\xe8\x7e\xff\x85\x0d\x72\x01\x00\xff\x8c\xff\xa7\xff\xb2\xd8\x00\x73\xcf\xff\xff\xf2\x40\x23\x15\x00\x01\x81\x41\xe0\x02\x81\x20\x08\xf7\x81\x34",
         50);
  _write(fd_i2c,
         "\xf2\x10\xa1\x40\xd8\x00\xc0\xd1\x7e\xe0\x53\x51\x30\x34\x20\x6f\x6e\x5f\x73\x74\x61\x72\x74\x5f\x73\x74\x72\x65\x61\x6d\x69\x6e\x67\x20\x25\x64\x20\x25\x64\x0a\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
         50);
  _write(fd_i2c, "\xf2\x40\xff\xff\xe8\x28\xff\xff\xf0\xe8\xff\xff\xe8\x08\xff\xff\xf1\x54", 18);
  _write(fd_i2c, "\x09\x8e\x00\x00", 4);
  _write(fd_i2c, "\xe0\x00\x05\xd8", 4);
  _write(fd_i2c, "\xe0\x02\x04\x03", 4);
  _write(fd_i2c, "\xe0\x04\x00\x43\x01\x04", 6);

  // Do while succeeded?
  _write(fd_i2c, "\x00\x40\x80\x01", 4);
  while (test[0] != 0xFF || test[1] != 0XF8) {
    tim.tv_sec = 0;
    tim.tv_nsec = 100000000;
    nanosleep(&tim, NULL);
    write(fd_i2c, "\x00\x40", 2);
    read(fd_i2c, test, 2);
    printf("Da: %X%X\n", test[0], test[1]);

    /*if(test[1] == 0XFB) {
      // restart all over??
      mt9v117_init();
      return;
    }*/
  }

  _write(fd_i2c, "\xac\x40\x00\x00\xc3\x50", 6);
  write_reg(fd_i2c, "\xa4\x04\x00\x00", 4);
  //write(fd_i2c, "\x00\x30", 2);
  //read(fd_i2c, "\x04\x00", 2);

  write_reg(fd_i2c, "\x00\x30\x06\x01", 4);
  write_reg(fd_i2c, "\xc8\x00\x00\x0c", 4); //0x0008
  write_reg(fd_i2c, "\xc8\x02\x00\x10", 4);
  write_reg(fd_i2c, "\xc8\x04\x01\xf3", 4); //0x01f5
  write_reg(fd_i2c, "\xc8\x06\x02\x97", 4);
  write_reg(fd_i2c, "\xc8\x08\x01\x11", 4);
  write_reg(fd_i2c, "\xc8\x0a\x00\xa4", 4);
  write_reg(fd_i2c, "\xc8\x0c\x02\xfa", 4);
  write_reg(fd_i2c, "\xc8\x12\x00\x31", 4);
  write_reg(fd_i2c, "\xc8\x14\x01\xe3", 4); //0x00f3
  write_reg(fd_i2c, "\xc8\x28\x00\x03", 4); //0x0007
  write_reg(fd_i2c, "\xc8\x4c\x02\x80", 4);
  write_reg(fd_i2c, "\xc8\x4e\x01\xe0", 4); //240 (0x00f0)
  write_reg(fd_i2c, "\xc8\x50\x03", 3);

  write_reg(fd_i2c, "\xc8\x54\x02\x80", 4); //320 (0x0140)
  write_reg(fd_i2c, "\xc8\x56\x01\xe0", 4); //240 (0x00f0)

  write_reg(fd_i2c, "\xc8\xec\x00\x00", 4);
  write_reg(fd_i2c, "\xc8\xee\x00\x00", 4);
  write_reg(fd_i2c, "\xc8\xf0\x02\x7f", 4); //0x013f
  write_reg(fd_i2c, "\xc8\xf2\x01\xdf", 4); //0x00ef
  write_reg(fd_i2c, "\xc8\xf4\x00\x02", 4);
  write_reg(fd_i2c, "\xc8\xf6\x00\x02", 4);
  write_reg(fd_i2c, "\xc8\xf8\x00\x7f", 4); //0x003f
  write_reg(fd_i2c, "\xc8\xfa\x00\x5f", 4); //0x002f
  write_reg(fd_i2c, "\xc8\x10\x03\x52", 4); //0x0400 (0x045e??)
  write_reg(fd_i2c, "\xc8\x0e\x01\xff", 4); //0x0140 (0x0143??)
  write_reg(fd_i2c, "\xc8\x16\x00\xd4", 4); //0x00b0 (0x00a1??)
  write_reg(fd_i2c, "\xc8\x18\x00\xfe", 4); //0x00d3 (0x00c1??)
  write_reg(fd_i2c, "\xc8\x1a\x00\x01", 4);
  write_reg(fd_i2c, "\xc8\x1c\x00\x02", 4); //0x0001
  write_reg(fd_i2c, "\xc8\x1e\x00\x01", 4);
  write_reg(fd_i2c, "\xc8\x20\x00\x02", 4); //0x0001

  write(fd_i2c, "\xc8\x58", 2);
  read(fd_i2c, test, 2);
  write_reg(fd_i2c, "\xc8\x58\x00\x18", 4);
  write_reg(fd_i2c, "\xdc\x00\x28", 3);

  // Dow while succeeded?
  _write(fd_i2c, "\x00\x40\x80\x02", 4);
  test[0] = 0;
  test[1] = 0;
  while (test[0] != 0xFF || test[1] != 0XF8) {
    tim.tv_sec = 0;
    tim.tv_nsec = 100000000;
    nanosleep(&tim, NULL);
    write(fd_i2c, "\x00\x40", 2);
    read(fd_i2c, test, 2);
    printf("Dt: %X%X\n", test[0], test[1]);
  }

  printf("Done!\n");
  close(fd_i2c);
}

int mt9f002_i2c_port;

void mt9f002_open(void);
void mt9f002_close(void);
void mt9f002_set_address(uint8_t address);
void mt9f002_write_reg8(uint16_t reg, uint8_t value);
void mt9f002_write_reg16(uint16_t reg, uint16_t value);
uint8_t mt9f002_read_reg8(uint16_t reg);
uint16_t mt9f002_read_reg16(uint16_t reg);

void mt9f002_open(void)
{
  mt9f002_i2c_port = open("/dev/i2c-0", O_RDWR);
  if (mt9f002_i2c_port < 0) {
    printf("mt9f002_open");
    // exit(1);
  }
}

void mt9f002_close(void)
{
  close(mt9f002_i2c_port);
}

void mt9f002_set_address(uint8_t address)
{
  if (ioctl(mt9f002_i2c_port, I2C_SLAVE_FORCE, address) < 0) {
    printf("mt9f002_set_address");
    // exit(1);
  }
}

void mt9f002_write_reg8(uint16_t reg, uint8_t value)
{
  mt9f002_open();
  mt9f002_set_address(0x10);

  uint8_t data[3];
  data[0] = (uint8_t)(reg >> 8) ;
  data[1] = (uint8_t)reg & 0xFF;
  data[2] = value;
  // if(i2c_smbus_write_byte_data(mt9f002_i2c_port,reg,value) < 0) {
  if (write(mt9f002_i2c_port, data, 3) < 0) {
    printf("mt9f002_write_reg8");
    // exit(1);
  }

  mt9f002_close();

  usleep(100);
}

void mt9f002_write_reg16(uint16_t reg, uint16_t value)
{
  mt9f002_open();
  mt9f002_set_address(0x10);

  uint8_t data[4];
  data[0] = (uint8_t)(reg >> 8);
  data[1] = (uint8_t)reg & 0xFF;
  data[2] = (uint8_t)(value >> 8);
  data[3] = value & 0xFF;
  if (write(mt9f002_i2c_port, data, 4) < 0) {
    printf("mt9f002_write_reg16");
    // exit(1);
  }

  mt9f002_close();

  usleep(100);
}

uint8_t mt9f002_read_reg8(uint16_t reg)
{
  mt9f002_open();
  mt9f002_set_address(0x10);

  uint8_t data[2];
  data[0] = (uint8_t)(reg >> 8);
  data[1] = (uint8_t)reg & 0xFF;
  write(mt9f002_i2c_port, data, 2);
  usleep(10);
  read(mt9f002_i2c_port, data, 1);

  mt9f002_close();

  return data[0];
}


uint16_t mt9f002_read_reg16(uint16_t reg)
{
  mt9f002_open();
  mt9f002_set_address(0x10);

  uint8_t data[2];
  data[0] = (uint8_t)(reg >> 8);
  data[1] = (uint8_t)reg & 0xFF;
  write(mt9f002_i2c_port, data, 2);
  usleep(10);
  read(mt9f002_i2c_port, data, 2);

  mt9f002_close();

  return data[1] | (data[0] << 8);
}

/**
 * Initialisation of the Aptina MT9F002 CMOS sensor
 * (1/2.3 inch 14Mp, front camera)
 */
void mt9f002_init(void)
{
  /* Change to standby mode */
  mt9f002_write_reg8(MT9F002_MODE_SELECT, 0x00);

  /* Change registers */
  mt9f002_write_reg16(MT9F002_P_GR_P0Q0, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P0Q2, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P0Q3, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P0Q4, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P0Q0, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P0Q1, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P0Q2, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P0Q3, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P0Q4, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P0Q0, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P0Q1, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P0Q2, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P0Q3, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P0Q4, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P0Q0, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P0Q1, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P0Q2, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P0Q3, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P0Q4, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P1Q0, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P1Q1, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P1Q2, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P1Q3, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P1Q4, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P1Q0, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P1Q1, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P1Q2, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P1Q3, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P1Q4, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P1Q0, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P1Q1, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P1Q2, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P1Q3, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P1Q4, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P1Q0, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P1Q1, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P1Q2, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P1Q3, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P1Q4, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P2Q0, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P2Q1, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P2Q2, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P2Q3, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P2Q4, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P2Q0, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P2Q1, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P2Q2, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P2Q3, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P2Q4, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P2Q0, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P2Q1, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P2Q2, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P2Q3, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P2Q4, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P2Q0, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P2Q1, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P2Q2, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P2Q3, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P2Q4, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P3Q0, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P3Q1, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P3Q2, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P3Q3, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P3Q4, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P3Q0, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P3Q1, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P3Q2, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P3Q3, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P3Q4, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P3Q0, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P3Q1, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P3Q2, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P3Q3, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P3Q4, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P3Q0, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P3Q1, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P3Q2, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P3Q3, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P3Q4, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P4Q0, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P4Q1, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P4Q2, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P4Q3, 0);
  mt9f002_write_reg16(MT9F002_P_GR_P4Q4, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P4Q0, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P4Q1, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P4Q2, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P4Q3, 0);
  mt9f002_write_reg16(MT9F002_P_RD_P4Q4, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P4Q0, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P4Q1, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P4Q2, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P4Q3, 0);
  mt9f002_write_reg16(MT9F002_P_BL_P4Q4, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P4Q0, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P4Q1, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P4Q2, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P4Q3, 0);
  mt9f002_write_reg16(MT9F002_P_GB_P4Q4, 0);

  mt9f002_write_reg16(MT9F002_GREEN1_GAIN, 4176);
  mt9f002_write_reg16(MT9F002_BLUE_GAIN, 4176);
  mt9f002_write_reg16(MT9F002_RED_GAIN, 4176);
  mt9f002_write_reg16(MT9F002_GREEN2_GAIN, 4176);
  mt9f002_write_reg16(MT9F002_GLOBAL_GAIN, 4176);
  mt9f002_write_reg16(MT9F002_ANALOG_GAIN_CODE_GLOBAL, 10);
  mt9f002_write_reg16(MT9F002_ANALOG_GAIN_CODE_GREENR, 10);
  mt9f002_write_reg16(MT9F002_ANALOG_GAIN_CODE_RED, 10);
  mt9f002_write_reg16(MT9F002_ANALOG_GAIN_CODE_BLUE, 10);
  mt9f002_write_reg16(MT9F002_ANALOG_GAIN_CODE_GREENB, 10);
  mt9f002_write_reg16(MT9F002_CALIB_GREEN1_ASC1, 4224);
  mt9f002_write_reg16(MT9F002_CALIB_BLUE_ASC1, 4224);
  mt9f002_write_reg16(MT9F002_CALIB_RED_ASC1, 4224);
  mt9f002_write_reg16(MT9F002_CALIB_GREEN2_ASC1, 4224);
  mt9f002_write_reg16(MT9F002_CALIB_GREEN1, 4224);
  mt9f002_write_reg16(MT9F002_CALIB_BLUE, 4224);
  mt9f002_write_reg16(MT9F002_CALIB_RED, 4224);
  mt9f002_write_reg16(MT9F002_CALIB_GREEN2, 4224);
  mt9f002_write_reg16(MT9F002_POLY_ORIGIN_C, 0);
  mt9f002_write_reg16(MT9F002_POLY_ORIGIN_R, 0);
  mt9f002_write_reg16(MT9F002_P_GR_Q5, 0);
  mt9f002_write_reg16(MT9F002_P_RD_Q5, 0);
  mt9f002_write_reg16(MT9F002_P_BL_Q5, 0);
  mt9f002_write_reg16(MT9F002_P_GB_Q5, 0);
  mt9f002_write_reg16(MT9F002_DAC_ID_FBIAS, 57568);

  mt9f002_write_reg16(MT9F002_FRAME_LENGTH_LINES, 3434);
  mt9f002_write_reg16(MT9F002_LINE_LENGTH_PCK, 2504);
  mt9f002_write_reg16(MT9F002_ROW_SPEED, 273);
  mt9f002_write_reg16(MT9F002_MIPI_TIMING_2, 61452);
  mt9f002_write_reg16(MT9F002_HISPI_TIMING, 0);

  mt9f002_write_reg16(MT9F002_VT_PIX_CLK_DIV, 6);
  mt9f002_write_reg16(MT9F002_PRE_PLL_CLK_DIV, 6);
  mt9f002_write_reg16(MT9F002_PLL_MULTIPLIER, 165);
  mt9f002_write_reg16(MT9F002_OP_PIX_CLK_DIV, 12);

  mt9f002_write_reg16(MT9F002_X_ADDR_START_, 500);
  mt9f002_write_reg16(MT9F002_Y_ADDR_START_, 32);
  mt9f002_write_reg16(MT9F002_X_ADDR_END_, 4527); // 4383 cols
  mt9f002_write_reg16(MT9F002_Y_ADDR_END_, 3319); // 3287 rows
  mt9f002_write_reg16(MT9F002_READ_MODE, 0x06E7);
  mt9f002_write_reg16(MT9F002_X_ODD_INC, 3);
  mt9f002_write_reg16(MT9F002_Y_ODD_INC, 3); // Sample 1, skip 7 -> /8

  mt9f002_write_reg16(MT9F002_SCALING_MODE, 2);
  mt9f002_write_reg16(MT9F002_SCALE_M, 48);


  /* Stream mode */
  mt9f002_write_reg8(MT9F002_MODE_SELECT, 0x01); // Stream mode on

}

#pragma GCC diagnostic pop /* end disable -Wunused-result */
