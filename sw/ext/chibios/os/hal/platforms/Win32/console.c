/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file console.c
 * @brief Simulator console driver code.
 * @{
 */

#include <stdio.h>

#include "ch.h"
#include "hal.h"
#include "console.h"

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief Console driver 1.
 */
BaseChannel CD1;

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static size_t write(void *ip, const uint8_t *bp, size_t n) {
  size_t ret;

  (void)ip;
  ret = fwrite(bp, 1, n, stdout);
  fflush(stdout);
  return ret;
}

static size_t read(void *ip, uint8_t *bp, size_t n) {

  (void)ip;
  return fread(bp, 1, n, stdin);
}

static msg_t put(void *ip, uint8_t b) {

  (void)ip;

  fputc(b, stdout);
  fflush(stdout);
  return RDY_OK;
}

static msg_t get(void *ip) {

  (void)ip;

  return fgetc(stdin);
}

static msg_t putt(void *ip, uint8_t b, systime_t time) {

  (void)ip;
  (void)time;
  fputc(b, stdout);
  fflush(stdout);
  return RDY_OK;
}

static msg_t gett(void *ip, systime_t time) {

  (void)ip;
  (void)time;
  return fgetc(stdin);
}

static size_t writet(void *ip, const uint8_t *bp, size_t n, systime_t time) {
  size_t ret;

  (void)ip;
  (void)time;
  ret = fwrite(bp, 1, n, stdout);
  fflush(stdout);
  return ret;
}

static size_t readt(void *ip, uint8_t *bp, size_t n, systime_t time) {

  (void)ip;
  (void)time;
  return fread(bp, 1, n, stdin);
}

static const struct BaseChannelVMT vmt = {
  write, read, put, get,
  putt, gett, writet, readt
};

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void conInit(void) {

  CD1.vmt = &vmt;
}

/** @} */
