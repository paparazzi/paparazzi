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

#include <stdio.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

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
  _write(fd_i2c, "\xf0\x00\x72\xcf\xff\x00\x3e\xd0\x92\x00\x71\xcf\xff\xff\xf2\x18\xb1\x10\x92\x05\xb1\x11\x92\x04\xb1\x12\x70\xcf\xff\x00\x30\xc0\x90\x00\x7f\xe0\xb1\x13\x70\xcf\xff\xff\xe7\x1c\x88\x36\x09\x0f\x00\xb3", 50);
  _write(fd_i2c, "\xf0\x30\x69\x13\xe1\x80\xd8\x08\x20\xca\x03\x22\x71\xcf\xff\xff\xe5\x68\x91\x35\x22\x0a\x1f\x80\xff\xff\xf2\x18\x29\x05\x00\x3e\x12\x22\x11\x01\x21\x04\x0f\x81\x00\x00\xff\xf0\x21\x8c\xf0\x10\x1a\x22", 50);
  _write(fd_i2c, "\xf0\x60\x10\x44\x12\x20\x11\x02\xf7\x87\x22\x4f\x03\x83\x1a\x20\x10\xc4\xf0\x09\xba\xae\x7b\x50\x1a\x20\x10\x84\x21\x45\x01\xc1\x1a\x22\x10\x44\x70\xcf\xff\x00\x3e\xd0\xb0\x60\xb0\x25\x7e\xe0\x78\xe0", 50);
  _write(fd_i2c, "\xf0\x90\x71\xcf\xff\xff\xf2\x18\x91\x12\x72\xcf\xff\xff\xe7\x1c\x8a\x57\x20\x04\x0f\x80\x00\x00\xff\xf0\xe2\x80\x20\xc5\x01\x61\x20\xc5\x03\x22\xb1\x12\x71\xcf\xff\x00\x3e\xd0\xb1\x04\x7e\xe0\x78\xe0", 50);
  _write(fd_i2c, "\xf0\xc0\x70\xcf\xff\xff\xe7\x1c\x88\x57\x71\xcf\xff\xff\xf2\x18\x91\x13\xea\x84\xb8\xa9\x78\x10\xf0\x03\xb8\x89\xb8\x8c\xb1\x13\x71\xcf\xff\x00\x30\xc0\xb1\x00\x7e\xe0\xc0\xf1\x09\x1e\x03\xc0\xc1\xa1", 50);
  _write(fd_i2c, "\xf0\xf0\x75\x08\x76\x28\x77\x48\xc2\x40\xd8\x20\x71\xcf\x00\x03\x20\x67\xda\x02\x08\xae\x03\xa0\x73\xc9\x0e\x25\x13\xc0\x0b\x5e\x01\x60\xd8\x06\xff\xbc\x0c\xce\x01\x00\xd8\x00\xb8\x9e\x0e\x5a\x03\x20", 50);
  _write(fd_i2c, "\xf1\x20\xd9\x01\xd8\x00\xb8\x9e\x0e\xb6\x03\x20\xd9\x01\x8d\x14\x08\x17\x01\x91\x8d\x16\xe8\x07\x0b\x36\x01\x60\xd8\x07\x0b\x52\x01\x60\xd8\x11\x8d\x14\xe0\x87\xd8\x00\x20\xca\x02\x62\x00\xc9\x03\xe0", 50);
  _write(fd_i2c, "\xf1\x50\xc0\xa1\x78\xe0\xc0\xf1\x08\xb2\x03\xc0\x76\xcf\xff\xff\xe5\x40\x75\xcf\xff\xff\xe5\x68\x95\x17\x96\x40\x77\xcf\xff\xff\xe5\x42\x95\x38\x0a\x0d\x00\x01\x97\x40\x0a\x11\x00\x40\x0b\x0a\x01\x00", 50);
  _write(fd_i2c, "\xf1\x80\x95\x17\xb6\x00\x95\x18\xb7\x00\x76\xcf\xff\xff\xe5\x44\x96\x20\x95\x15\x08\x13\x00\x40\x0e\x1e\x01\x20\xd9\x00\x95\x15\xb6\x00\xff\xa1\x75\xcf\xff\xff\xe7\x1c\x77\xcf\xff\xff\xe5\x46\x97\x40", 50);
  _write(fd_i2c, "\xf1\xb0\x8d\x16\x76\xcf\xff\xff\xe5\x48\x8d\x37\x08\x0d\x00\x81\x96\x40\x09\x15\x00\x80\x0f\xd6\x01\x00\x8d\x16\xb7\x00\x8d\x17\xb6\x00\xff\xb0\xff\xbc\x00\x41\x03\xc0\xc0\xf1\x0d\x9e\x01\x00\xe8\x04", 50);
  _write(fd_i2c, "\xf1\xe0\xff\x88\xf0\x0a\x0d\x6a\x01\x00\x0d\x8e\x01\x00\xe8\x7e\xff\x85\x0d\x72\x01\x00\xff\x8c\xff\xa7\xff\xb2\xd8\x00\x73\xcf\xff\xff\xf2\x40\x23\x15\x00\x01\x81\x41\xe0\x02\x81\x20\x08\xf7\x81\x34", 50);
  _write(fd_i2c, "\xf2\x10\xa1\x40\xd8\x00\xc0\xd1\x7e\xe0\x53\x51\x30\x34\x20\x6f\x6e\x5f\x73\x74\x61\x72\x74\x5f\x73\x74\x72\x65\x61\x6d\x69\x6e\x67\x20\x25\x64\x20\x25\x64\x0a\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00", 50);
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
  write_reg(fd_i2c, "\xc8\x00\x00\x08", 4);
  write_reg(fd_i2c, "\xc8\x02\x00\x10", 4);
  write_reg(fd_i2c, "\xc8\x04\x01\xf5", 4);
  write_reg(fd_i2c, "\xc8\x06\x02\x97", 4);
  write_reg(fd_i2c, "\xc8\x08\x01\x11", 4);
  write_reg(fd_i2c, "\xc8\x0a\x00\xa4", 4);
  write_reg(fd_i2c, "\xc8\x0c\x02\xfa", 4);
  write_reg(fd_i2c, "\xc8\x12\x00\x31", 4);
  write_reg(fd_i2c, "\xc8\x14\x00\xf3", 4);
  write_reg(fd_i2c, "\xc8\x28\x00\x07", 4);
  write_reg(fd_i2c, "\xc8\x4c\x02\x80", 4);
  write_reg(fd_i2c, "\xc8\x4e\x00\xf0", 4);
  write_reg(fd_i2c, "\xc8\x50\x03", 3);
  write_reg(fd_i2c, "\xc8\x54\x01\x40", 4);
  write_reg(fd_i2c, "\xc8\x56\x00\xf0", 4);
  write_reg(fd_i2c, "\xc8\xec\x00\x00", 4);
  write_reg(fd_i2c, "\xc8\xee\x00\x00", 4);
  write_reg(fd_i2c, "\xc8\xf0\x01\x3f", 4);
  write_reg(fd_i2c, "\xc8\xf2\x00\xef", 4);
  write_reg(fd_i2c, "\xc8\xf4\x00\x02", 4);
  write_reg(fd_i2c, "\xc8\xf6\x00\x02", 4);
  write_reg(fd_i2c, "\xc8\xf8\x00\x3f", 4);
  write_reg(fd_i2c, "\xc8\xfa\x00\x2f", 4);
  write_reg(fd_i2c, "\xc8\x10\x04\x00", 4);
  write_reg(fd_i2c, "\xc8\x0e\x01\x40", 4);
  write_reg(fd_i2c, "\xc8\x16\x00\xb0", 4);
  write_reg(fd_i2c, "\xc8\x18\x00\xd3", 4);
  write_reg(fd_i2c, "\xc8\x1a\x00\x01", 4);
  write_reg(fd_i2c, "\xc8\x1c\x00\x01", 4);
  write_reg(fd_i2c, "\xc8\x1e\x00\x01", 4);
  write_reg(fd_i2c, "\xc8\x20\x00\x01", 4);
  //write_reg(fd_i2c, "\xc8\x58", 2);
  //read(fd_i2c, "\x00\x10", 2);
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

/**
 * Initialisation of the Aptina MT9F002 CMOS sensor
 */
void mt9f002_init(void)
{
  /* We open the i2c-0 (because else I needed to convert the strace) */
  int fd_i2c = open("/dev/i2c-0", O_RDWR);
  if (fd_i2c < 0) {
    printf("[MT9F002] Could not open i2c-0\n");
    return;
  }
  if (ioctl(fd_i2c, 0x706, 0x10) < 0) {
    printf("[MT9V117] Could not change the i2c address to 0x10\n");
    return;
  }

  /* First reset the device */
  write(fd_i2c, "\1\3\1", 3);
  //usleep(1);

  write(fd_i2c, "0\32\0\20", 4);
  write(fd_i2c, "0^\0240", 4);
  write(fd_i2c, "0\32\0\20", 4);
  write(fd_i2c, "0\32\0\20", 4);
  write(fd_i2c, "0\32\0\20", 4);
  write(fd_i2c, ">\332\345%", 4);
  write(fd_i2c, "0\350\0\0", 4);
  write(fd_i2c, "0\352\370s", 4);
  write(fd_i2c, "0\352\10\252", 4);
  write(fd_i2c, "0\3522\31", 4);
  write(fd_i2c, "0\3522\31", 4);
  write(fd_i2c, "0\3522\31", 4);
  write(fd_i2c, "0\3522\0", 4);
  write(fd_i2c, "0\3522\0", 4);
  write(fd_i2c, "0\3522\0", 4);
  write(fd_i2c, "0\3522\0", 4);
  write(fd_i2c, "0\3522\0", 4);
  write(fd_i2c, "0\352\27i", 4);
  write(fd_i2c, "0\352\246\363", 4);
  write(fd_i2c, "0\352\246\363", 4);
  write(fd_i2c, "0\352\246\363", 4);
  write(fd_i2c, "0\352\246\363", 4);
  write(fd_i2c, "0\352\246\363", 4);
  write(fd_i2c, "0\352\246\363", 4);
  write(fd_i2c, "0\352\246\363", 4);
  write(fd_i2c, "0\352\257\363", 4);
  write(fd_i2c, "0\352\372d", 4);
  write(fd_i2c, "0\352\372d", 4);
  write(fd_i2c, "0\352\372d", 4);
  write(fd_i2c, "0\352\361d", 4);
  write(fd_i2c, "0\352\372d", 4);
  write(fd_i2c, "0\352\372d", 4);
  write(fd_i2c, "0\352\372d", 4);
  write(fd_i2c, "0\352\361d", 4);
  write(fd_i2c, "0\352'n", 4);
  write(fd_i2c, "0\352\30\317", 4);
  write(fd_i2c, "0\352\30\317", 4);
  write(fd_i2c, "0\352\30\317", 4);
  write(fd_i2c, "0\352(\317", 4);
  write(fd_i2c, "0\352\30\317", 4);
  write(fd_i2c, "0\352\30\317", 4);
  write(fd_i2c, "0\352\30\317", 4);
  write(fd_i2c, "0\352\30\317", 4);
  write(fd_i2c, "0\352#c", 4);
  write(fd_i2c, "0\352#c", 4);
  write(fd_i2c, "0\352#R", 4);
  write(fd_i2c, "0\352#c", 4);
  write(fd_i2c, "0\352#c", 4);
  write(fd_i2c, "0\352#c", 4);
  write(fd_i2c, "0\352#R", 4);
  write(fd_i2c, "0\352#R", 4);
  write(fd_i2c, "0\352\243\224", 4);
  write(fd_i2c, "0\352\243\224", 4);
  write(fd_i2c, "0\352\217\217", 4);
  write(fd_i2c, "0\352\243\324", 4);
  write(fd_i2c, "0\352\243\224", 4);
  write(fd_i2c, "0\352\243\224", 4);
  write(fd_i2c, "0\352\217\217", 4);
  write(fd_i2c, "0\352\217\317", 4);
  write(fd_i2c, "0\352\334#", 4);
  write(fd_i2c, "0\352\334c", 4);
  write(fd_i2c, "0\352\334c", 4);
  write(fd_i2c, "0\352\334#", 4);
  write(fd_i2c, "0\352\334#", 4);
  write(fd_i2c, "0\352\334c", 4);
  write(fd_i2c, "0\352\334c", 4);
  write(fd_i2c, "0\352\334#", 4);
  write(fd_i2c, "0\352\17s", 4);
  write(fd_i2c, "0\352\205\300", 4);
  write(fd_i2c, "0\352\205\300", 4);
  write(fd_i2c, "0\352\205\300", 4);
  write(fd_i2c, "0\352\205\300", 4);
  write(fd_i2c, "0\352\205\300", 4);
  write(fd_i2c, "0\352\205\300", 4);
  write(fd_i2c, "0\352\205\300", 4);
  write(fd_i2c, "0\352\205\304", 4);
  write(fd_i2c, "0\352\0\0", 4);
  write(fd_i2c, "1v\200\0", 4);
  write(fd_i2c, ">\332\345%", 4);
  write(fd_i2c, "0\36\0\250", 4);
  write(fd_i2c, "0\32\0\220", 4);
  write(fd_i2c, "1\256\3\1", 4);
  write(fd_i2c, "0\32\20\220", 4);
  write(fd_i2c, "0d\10E", 4);
  write(fd_i2c, "0\32\20\200", 4);
  write(fd_i2c, "0n\330\200", 4);
  write(fd_i2c, "0\32\220\200", 4);
  write(fd_i2c, "0n\330\200", 4);
  write(fd_i2c, "0\32\20\310", 4);
  write(fd_i2c, "0n\330\200", 4);
  write(fd_i2c, "\3\0\0\7", 4);
  write(fd_i2c, "\3\2\0\1", 4);
  write(fd_i2c, "\3\4\0\1", 4);
  write(fd_i2c, "\3\6\0;", 4);
  write(fd_i2c, "\3\10\0\10", 4);
  write(fd_i2c, "\3\n\0\1", 4);


  ioctl(fd_i2c, 0x707, 0xd0);
  write(fd_i2c, "0d\10E", 4);

  ioctl(fd_i2c, 0x707, 0xd0);
  write(fd_i2c, "0\26\1!", 4);
  write(fd_i2c, "1v\200\0", 4);
  write(fd_i2c, "0@\0A", 4);
  write(fd_i2c, "0@\4\303", 4);
  write(fd_i2c, "0@\4\303", 4);
  write(fd_i2c, "1x\0\0", 4);
  write(fd_i2c, "1x\0\0", 4);
  write(fd_i2c, "1x\0\0", 4);
  write(fd_i2c, "1x\0\0", 4);
  write(fd_i2c, ">\350\0G", 4);
  write(fd_i2c, "0\324\260\200", 4);
  write(fd_i2c, "0\324\261\0", 4);
  write(fd_i2c, "0\356\0 ", 4);
  write(fd_i2c, ">\344cI", 4);
  write(fd_i2c, "1|\200\n", 4);
  write(fd_i2c, "0\32\220\310", 4);
  write(fd_i2c, "0\350\200\5", 4);
  write(fd_i2c, "1|\200\n", 4);
  write(fd_i2c, ">\350\0G", 4);
  write(fd_i2c, ">\352\25\360", 4);
  write(fd_i2c, ">\352\25\360", 4);
  write(fd_i2c, ">\352\25\360", 4);
  write(fd_i2c, ">\350\0G", 4);
  write(fd_i2c, ">\350\0G", 4);
  write(fd_i2c, "0\32\20\310", 4);
  write(fd_i2c, "\2\2\10\303", 4);
  write(fd_i2c, "0\260\0\0", 4);
  write(fd_i2c, "0n\330\200", 4);
  write(fd_i2c, "0@\0A", 4);
  write(fd_i2c, "\1\5\1", 3);
  write(fd_i2c, "\3L\2\200", 4);
  write(fd_i2c, "\3N\1\340", 4);
  write(fd_i2c, "\4\0\0\0", 4);
  write(fd_i2c, "0@\0A", 4);
  write(fd_i2c, "\3\202\0\1", 4);
  write(fd_i2c, "\3\206\0\1", 4);
  write(fd_i2c, "\3D\0\0", 4);
  write(fd_i2c, "\3H\2\177", 4);
  write(fd_i2c, "\3F\0\0", 4);
  write(fd_i2c, "\3J\1\337", 4);

  write(fd_i2c, "\3B&\340", 4);
  write(fd_i2c, "\3@\2\336", 4);
  write(fd_i2c, "\2\2\2\224", 4);
  write(fd_i2c, "0\24\27M", 4);
  write(fd_i2c, "0V\24@", 4);
  write(fd_i2c, "0X\24@", 4);
  write(fd_i2c, "0Z\24@", 4);
  write(fd_i2c, "0\\\24@", 4);
  write(fd_i2c, "\1\4\1", 3);
  write(fd_i2c, "\3L\2\200", 4);
  write(fd_i2c, "\3N\1h", 4);
  write(fd_i2c, "\4\0\0\0", 4);
  write(fd_i2c, "0@\0A", 4);
  write(fd_i2c, "\3\202\0\1", 4);
  write(fd_i2c, "\3\206\0\1", 4);
  write(fd_i2c, "\3D\0\0", 4);
  write(fd_i2c, "\3H\2\177", 4);
  write(fd_i2c, "\3F\0\0", 4);
  write(fd_i2c, "\3J\1g", 4);

  write(fd_i2c, "\3B&\340", 4);
  write(fd_i2c, "\3@\2\336", 4);
  write(fd_i2c, "\2\2\2\224", 4);
  write(fd_i2c, "0\24\27M", 4);
  write(fd_i2c, "\1\4\0", 3);

  write(fd_i2c, "\1\4\1", 3);
  write(fd_i2c, "\3B&\340", 4);
  write(fd_i2c, "\3@\2\336", 4);
  write(fd_i2c, "\2\2\2\224", 4);
  write(fd_i2c, "0\24\27M", 4);
  write(fd_i2c, "\1\4\0", 3);
  write(fd_i2c, "\1\4\1", 3);
  write(fd_i2c, "\2\2\0\334", 4);
  write(fd_i2c, "0\24\7\304", 4);
  write(fd_i2c, "\1\4\0", 3);
  write(fd_i2c, "0VL@", 4);
  write(fd_i2c, "0XL@", 4);
  write(fd_i2c, "0ZL@", 4);
  write(fd_i2c, "0\\L@", 4);
  write(fd_i2c, "\3N\0\0", 4);
  write(fd_i2c, "\1\0\1", 3);
  write(fd_i2c, "\1\4\1", 3);
  write(fd_i2c, "\3D\7\320", 4);
  write(fd_i2c, "\3H\nO", 4);
  write(fd_i2c, "\3F\5\310", 4);
  write(fd_i2c, "\3J\7/", 4);
  write(fd_i2c, "\1\4\0", 3);

  //usleep(1);
  write(fd_i2c, "\3N\1h", 4);
  write(fd_i2c, "\1\0\1", 3);
  close(fd_i2c);
}

#pragma GCC diagnostic pop /* end disable -Wunused-result */
