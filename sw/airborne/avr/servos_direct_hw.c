/* Implementation of command.h */

/* 
   3 servos on OC3A OC3B OC3C using Timer3 prescaled at 8
*/
#include "std.h"
#include "command.h"

#include "airframe.h"

const pprz_t failsafe_values[COMMANDS_NB] = COMMANDS_FAILSAFE;

void command_set(const pprz_t values[]) {
  CommandsSet(values); /*Generated from airframe.xml */
}
