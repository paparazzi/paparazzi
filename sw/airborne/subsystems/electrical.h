#ifndef SUBSYSTEMS_ELECTRICAL_H
#define SUBSYSTEMS_ELECTRICAL_H

#include "std.h"

#define LowBatLevel()	(electrical.vsupply_low == 0)

struct Electrical {

  uint16_t vsupply; /* supply in decivolts */
  int32_t current; /* current in milliamps */
  int32_t consumed; /* consumption in mAh */
	uint16_t vsupply_low; /* low bat voltage symbol */

};

extern struct Electrical electrical;

extern void electrical_init(void);
extern void electrical_periodic(void);

#endif /* SUBSYSTEMS_ELECTRICAL_H */
