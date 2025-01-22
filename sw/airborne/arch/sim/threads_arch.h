#pragma once

#include <pthread.h>
#include <semaphore.h>

#define PPRZ_NORMAL_PRIO 128

struct pprzMutex {
  pthread_mutex_t mtx;
};

struct pprzBSem {
  pthread_mutex_t mtx;
  sem_t sem;
};


typedef pthread_t pprz_thread_t;
