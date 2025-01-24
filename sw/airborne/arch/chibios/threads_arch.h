#pragma once

#include <ch.h>

#define PPRZ_NORMAL_PRIO NORMALPRIO

struct pprzMutex {
  mutex_t mtx;
};

struct pprzBSem {
  binary_semaphore_t bsem;
};


typedef thread_t* pprz_thread_t;
