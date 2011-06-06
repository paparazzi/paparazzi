#ifndef SUBSYSTEMS_ELECTRICAL_H
#define SUBSYSTEMS_ELECTRICAL_H

#include "std.h"

struct Electrical {

  uint8_t vsupply; /* supply in decivolts */
  int32_t current; /* current in milliamps */

};

extern struct Electrical electrical;

extern void electrical_init(void);
extern void electrical_periodic(void);

#endif /* SUBSYSTEMS_ELECTRICAL_H */
