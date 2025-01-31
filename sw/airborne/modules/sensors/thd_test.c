/*
 * Copyright (C) 2025 fab <fab@github.com>
 *
 * This file is part of paparazzi
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

/** @file "modules/sensors/thd_test.c"
 * @author fab <fab@github.com>
 * A module to test threads functionnalities, like threads, mutexes, semaphores and so on.
 */

#include "modules/sensors/thd_test.h"
#include "modules/core/threads.h"
#include "led.h"
#include "modules/datalink/telemetry.h"
#include <stdio.h>

float thd_test_start_value;

pprz_bsem_t bsem;

pprz_mutex_t mtx;   // mutex protecting syracuse
static int syracuse = 14;

// Remember if the thread has already been joined or not.
// From https://linux.die.net/man/3/pthread_join:
// "Joining with a thread that has previously been joined results in undefined behavior."
static bool joined = true;

static void test_thd(void*);

pprz_thread_t thd_handle;

void thd_test_syracuse_restart(float s) {
  // try changing syracuse 
  // non blocking because we are in the main loop.
  // If it doesn't work, just try again!
  if(!pprz_mtx_trylock(&mtx)) {
    syracuse = (int)s;
    thd_test_start_value = s;
    pprz_mtx_unlock(&mtx);

    // if it was stopped, restart it!
    if(joined) {
      if(!pprz_thread_create(&thd_handle, 512, "test", PPRZ_NORMAL_PRIO+1, test_thd, NULL)) {
        joined = false;
      }
    }
  } else {
    char err[] = "Could not lock";
    DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(err), err);
  }
}

void thd_test_init(void)
{
  pprz_bsem_init(&bsem, true);  // init binary semaphore
  pprz_mtx_init(&mtx);          // init mutex
  // create thread, and remember to try joining it.
  if(!pprz_thread_create(&thd_handle, 512, "test", PPRZ_NORMAL_PRIO, test_thd, NULL)) {
    joined = false;
  }
}

void thd_test_periodic(void)
{
  if(!joined) {
    size_t ret;
    char msg[30];
    size_t len = 0;
    if(!pprz_thread_tryjoin(&thd_handle, (void**)(&ret))) {
      // try joining the thread. If successful, send its exit value.
      len = snprintf(msg, sizeof(msg), "Finished in %zu steps!", ret);
      joined = true;
    } else {
      // the thread is still running, print the current value of the syracuse suit (mutex protected)
      // then signal the binary semaphore to allow the thread to resume execution
      if(!pprz_mtx_trylock(&mtx)) {
        len = snprintf(msg, sizeof(msg), "%d", syracuse);
        pprz_mtx_unlock(&mtx);
        pprz_bsem_signal(&bsem);
      }
    }
    // send the message to the ground (current value or end message)
    if(len > 0) {
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, len, msg);
    }
    
    
  }
  
}


static void test_thd(void* arg) {
  (void)arg;
  size_t count = 0;
  while(syracuse != 1) {
    pprz_bsem_wait(&bsem);  // wait to be woken up by the AP thread

    pprz_mtx_lock(&mtx);
    if(syracuse%2) {
      syracuse = 3*syracuse + 1;
    } else {
      syracuse = syracuse / 2;
    }
    pprz_mtx_unlock(&mtx);
    count++;
  }

  pprz_thread_exit((void*)((size_t)count));
}
