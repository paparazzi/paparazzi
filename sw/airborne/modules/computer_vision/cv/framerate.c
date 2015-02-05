
#include "std.h"
#include "framerate.h"

// Frame Rate (FPS)
#include <sys/time.h>

// local variables
volatile long timestamp;
struct timeval start_time;
struct timeval end_time;

#define USEC_PER_SEC 1000000L

float FPS;

static long time_elapsed(struct timeval *t1, struct timeval *t2)
{
  long sec, usec;
  sec = t2->tv_sec - t1->tv_sec;
  usec = t2->tv_usec - t1->tv_usec;
  if (usec < 0) {
    --sec;
    usec = usec + USEC_PER_SEC;
  }
  return sec * USEC_PER_SEC + usec;
}

static void start_timer(void)
{
  gettimeofday(&start_time, NULL);
}

static long end_timer(void)
{
  gettimeofday(&end_time, NULL);
  return time_elapsed(&start_time, &end_time);
}


void framerate_init(void) {
  // Frame Rate Initialization
  FPS = 0.0;
  timestamp = 0;
  start_timer();
}

void framerate_run(void) {
  // FPS
  timestamp = end_timer();
  FPS = (float) 1000000 / (float)timestamp;
  //  printf("dt = %d, FPS = %f\n",timestamp, FPS);
  start_timer();
}
