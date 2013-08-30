#ifndef SUBSYSTEMS_ELECTRICAL_H
#define SUBSYSTEMS_ELECTRICAL_H

#include "std.h"

struct Electrical {

  uint16_t vsupply;       ///< supply voltage in decivolts
  int32_t  current;       ///< current in milliamps
  int32_t  consumed;      ///< consumption in mAh
  bool_t   bat_low;       ///< battery low status
  bool_t   bat_critic;  ///< battery critical status

};

extern struct Electrical electrical;

extern void electrical_init(void);
extern void electrical_periodic(void);

#endif /* SUBSYSTEMS_ELECTRICAL_H */
