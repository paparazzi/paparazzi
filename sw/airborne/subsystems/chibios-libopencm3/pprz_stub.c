#include "pprz_stub.h"
#include "subsystems/gps.h"

uint16_t getGpsWeek (void)
{
  return gps.week;
}

uint32_t getGpsTimeOfWeek (void)
{
  return gps.tow;
}
