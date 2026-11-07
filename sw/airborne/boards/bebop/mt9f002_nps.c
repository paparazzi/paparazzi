// NPS wrapper around mt9f002.c

#define front_camera mt9f002_front_camera // Do not overwrite NPS camera struct

#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-parameter"
// no sensor in the simulator: the driver only runs its configuration code, without I2C
#define MT9F002_NO_SENSOR
#include "mt9f002.c"
#pragma GCC diagnostic pop
