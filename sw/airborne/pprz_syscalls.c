/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * Fake system calls
 *
 * based on:
 * ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

/*
* **** This file incorporates work covered by the following copyright and ****
* **** permission notice:                                                 ****
*
*  Copyright (c) 2009 by Michael Fischer. All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
*
****************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>

/***************************************************************************/

#pragma GCC diagnostic ignored "-Wmissing-prototypes"

#ifndef USE_CHIBIOS_RTOS // already defined by chibios syscalls

__attribute__((used))
int _read_r(struct _reent *r, int file, char * ptr, int len)
{
  (void)r;
  (void)file;
  (void)ptr;
  (void)len;
  return -1;
}

/***************************************************************************/

__attribute__((used))
int _lseek_r(struct _reent *r, int file, int ptr, int dir)
{
  (void)r;
  (void)file;
  (void)ptr;
  (void)dir;
  return 0;
}

/***************************************************************************/

__attribute__((used))
int _write_r(struct _reent *r, int file, char * ptr, int len)
{
  (void)r;
  (void)file;
  (void)ptr;
  return len;
}

/***************************************************************************/

__attribute__((used))
int _close_r(struct _reent *r, int file)
{
  (void)r;
  (void)file;
  return 0;
}

/***************************************************************************/

__attribute__((used))
caddr_t _sbrk_r(struct _reent *r, int incr)
{
  (void)r;
  (void)incr;
  return (caddr_t)-1;
}

/***************************************************************************/

__attribute__((used))
int _fstat_r(struct _reent *r, int file, struct stat * st)
{
  (void)r;
  (void)file;
  (void)st;
  return 0;
}

/***************************************************************************/

__attribute__((used))
int _isatty_r(struct _reent *r, int fd)
{
  (void)r;
  (void)fd;
  return 1;
}

#endif // USE_CHIBIOS_RTOS

/***************************************************************************/

__attribute__((used))
pid_t _getpid(void)
{
  return 1;
}

/***************************************************************************/

__attribute__((noreturn))
void _exit(int i) {
  (void)i;
  while(1);
}

/***************************************************************************/

void _kill(void) {}

#pragma GCC diagnostic pop

/*** EOF ***/
