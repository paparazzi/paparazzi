#ifndef HUMID_HTM_H
#define HUMID_HTM_H

#include "std.h"

enum htm_type {
  HTM_IDLE,
  HTM_MR,
  HTM_MR_OK,
  HTM_READ_DATA
};

void humid_htm_init(void);
void humid_htm_start(void);
void humid_htm_read(void);
void humid_htm_event(void);

#endif // HUMID_HTM_H

