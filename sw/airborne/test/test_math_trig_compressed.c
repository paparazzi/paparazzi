/*
 * Copyright (C) 2015 Martin Mueller <martinmm@pfump.org>
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
 * @file test_math_trig_compressed.c
 *
 * Run flash / RAM compressed sine look up table
 */

/*

 simple 4-bit run length encoding for sine table storage in flash
 reduces flash usage from 12868 bytes to 1690 bytes

 PPRZ_TRIG_INT_COMPR_FLASH

 0 0000: 0
 1 0001: 1
 2 0010: 2
 3 0011: 3
 4 0100: 4
 5 0101: 00
 6 0110: 11
 7 0111: 22
 8 1000: 33
 9 1001: 44
10 1010: 000
11 1011: 111
12 1100: 222
13 1101: 333
14 1110: 444
15 1111 nnnn nyyy: ((n+10) * y), n=00000..11111, y=0000..0100


 binary trees for sine table storage in RAM
 reduces RAM usage from 12868 bytes to

 PPRZ_TRIG_INT_COMPR_HIGHEST
   4 bit data + 10 bit tree   5265 bytes  (6434 *  4 + 1024 * 16 bits)
 PPRZ_TRIG_INT_COMPR_HIGH
   8 bit data +  6 bit tree   6562 bytes  (6434 *  8 +   64 * 16 bits)
 PPRZ_TRIG_INT_COMPR_LOW
  12 bit data                 9647 bytes  (6434 * 12)

 code size Cortex-M4 thumb2

 PPRZ_TRIG_INT_COMPR_HIGHEST  104 bytes
 PPRZ_TRIG_INT_COMPR_HIGH      76 bytes
 PPRZ_TRIG_INT_COMPR_LOW       84 bytes
 -                             12 bytes

 machine cycles STM32F4 (average over 10 calls)

 PPRZ_TRIG_INT_COMPR_HIGHEST  197 cycles
 PPRZ_TRIG_INT_COMPR_HIGH     170 cycles
 PPRZ_TRIG_INT_COMPR_LOW       46 cycles
 -                             16 cycles

*/

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "modules/datalink/downlink.h"
#include "led.h"
#include "math/pprz_trig_int.h"
#include "math/pprz_algebra_int.h"

/* cycle counter only exists for STM32 architecture */
#if defined(STM32F1) || defined(STM32F4)
#include <libopencm3/cm3/dwt.h>
#else
#define dwt_read_cycle_counter() 0
#define dwt_enable_cycle_counter() 0
#endif

static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);

int test_tables(void);

int32_t pprz_itrig_sin_4(int32_t angle);
int32_t pprz_itrig_sin_8(int32_t angle);
int32_t pprz_itrig_sin_12(int32_t angle);
int32_t pprz_itrig_sin_16(int32_t angle);

int test_tables(void)
{
  int16_t i;

  /* test all functions */
  for (i = 0; i < TRIG_INT_SIZE; i++) {
    if (pprz_trig_int[i] != pprz_trig_int_4(i)) return -1;
    if (pprz_trig_int[i] != pprz_trig_int_8(i)) return -1;
    if (pprz_trig_int[i] != pprz_trig_int_12(i)) return -1;
    if (pprz_trig_int[i] != pprz_trig_int_16(i)) return -1;
  }
  return 0;
}

#ifdef TRIG_INT_COMPR_4
int32_t pprz_itrig_sin_4(int32_t angle)
{
  INT32_ANGLE_NORMALIZE(angle);
  if (angle > INT32_ANGLE_PI_2) {
    angle = INT32_ANGLE_PI - angle;
  } else if (angle < -INT32_ANGLE_PI_2) {
    angle = -INT32_ANGLE_PI - angle;
  }
  if (angle >= 0) {
    return pprz_trig_int_4(angle);
  } else {
    return -pprz_trig_int_4(-angle);
  }
}
#endif

#ifdef TRIG_INT_COMPR_8
int32_t pprz_itrig_sin_8(int32_t angle)
{
  INT32_ANGLE_NORMALIZE(angle);
  if (angle > INT32_ANGLE_PI_2) {
    angle = INT32_ANGLE_PI - angle;
  } else if (angle < -INT32_ANGLE_PI_2) {
    angle = -INT32_ANGLE_PI - angle;
  }
  if (angle >= 0) {
    return pprz_trig_int_8(angle);
  } else {
    return -pprz_trig_int_8(-angle);
  }
}
#endif

#ifdef TRIG_INT_COMPR_12
int32_t pprz_itrig_sin_12(int32_t angle)
{
  INT32_ANGLE_NORMALIZE(angle);
  if (angle > INT32_ANGLE_PI_2) {
    angle = INT32_ANGLE_PI - angle;
  } else if (angle < -INT32_ANGLE_PI_2) {
    angle = -INT32_ANGLE_PI - angle;
  }
  if (angle >= 0) {
    return pprz_trig_int_12(angle);
  } else {
    return -pprz_trig_int_12(-angle);
  }
}
#endif

int32_t pprz_itrig_sin_16(int32_t angle)
{
  INT32_ANGLE_NORMALIZE(angle);
  if (angle > INT32_ANGLE_PI_2) {
    angle = INT32_ANGLE_PI - angle;
  } else if (angle < -INT32_ANGLE_PI_2) {
    angle = -INT32_ANGLE_PI - angle;
  }
  if (angle >= 0) {
    return pprz_trig_int_16(angle);
  } else {
    return -pprz_trig_int_16(-angle);
  }
}

int main(void)
{
  main_init();

  dwt_enable_cycle_counter();

  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic();
    }
    main_event();
  }
  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  datalink_init();
  downlink_init();
}

static inline void main_periodic(void)
{
  volatile uint32_t result;
  volatile uint32_t pre_time, post_time;
  uint32_t result1=0, result2=0, result3=0, result4=0;
  static int16_t i = 2342;

  i += TRIG_INT_SIZE / 4;
  i = i % TRIG_INT_SIZE;

  RunOnceEvery(10, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);});
  LED_PERIODIC();

  pprz_trig_int_init();
  if (test_tables() == 0) {

    /* run 10x without a loop */
    pre_time = dwt_read_cycle_counter();
    result = pprz_trig_int_4(i);
    result = pprz_trig_int_4(i);
    result = pprz_trig_int_4(i);
    result = pprz_trig_int_4(i);
    result = pprz_trig_int_4(i);
    result = pprz_trig_int_4(i);
    result = pprz_trig_int_4(i);
    result = pprz_trig_int_4(i);
    result = pprz_trig_int_4(i);
    result = pprz_trig_int_4(i);
    post_time = dwt_read_cycle_counter();
    result1 = (post_time - pre_time);

    pre_time = dwt_read_cycle_counter();
    result = pprz_trig_int_8(i);
    result = pprz_trig_int_8(i);
    result = pprz_trig_int_8(i);
    result = pprz_trig_int_8(i);
    result = pprz_trig_int_8(i);
    result = pprz_trig_int_8(i);
    result = pprz_trig_int_8(i);
    result = pprz_trig_int_8(i);
    result = pprz_trig_int_8(i);
    result = pprz_trig_int_8(i);
    post_time = dwt_read_cycle_counter();
    result2 = (post_time - pre_time);

    pre_time = dwt_read_cycle_counter();
    result = pprz_trig_int_12(i);
    result = pprz_trig_int_12(i);
    result = pprz_trig_int_12(i);
    result = pprz_trig_int_12(i);
    result = pprz_trig_int_12(i);
    result = pprz_trig_int_12(i);
    result = pprz_trig_int_12(i);
    result = pprz_trig_int_12(i);
    result = pprz_trig_int_12(i);
    result = pprz_trig_int_12(i);
    post_time = dwt_read_cycle_counter();
    result3 = (post_time - pre_time);

    pre_time = dwt_read_cycle_counter();
    result = pprz_trig_int_16(i);
    result = pprz_trig_int_16(i);
    result = pprz_trig_int_16(i);
    result = pprz_trig_int_16(i);
    result = pprz_trig_int_16(i);
    result = pprz_trig_int_16(i);
    result = pprz_trig_int_16(i);
    result = pprz_trig_int_16(i);
    result = pprz_trig_int_16(i);
    result = pprz_trig_int_16(i);
    post_time = dwt_read_cycle_counter();
    result4 = (post_time - pre_time);

    result = result;

    DOWNLINK_SEND_CSC_CAN_MSG(DefaultChannel, DefaultDevice, &result1, &result2, &result3, &result4);
  }
}

static inline void main_event(void)
{
  mcu_event();
}
