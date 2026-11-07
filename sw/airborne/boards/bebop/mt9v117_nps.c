// NPS wrapper for mt9v117.c

#define bottom_camera mt9v117_bottom_camera // Do not overwrite NPS camera struct

#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-parameter"
// no sensor in the simulator: the driver only runs its configuration code, without I2C
#define MT9V117_NO_SENSOR
#include "mt9v117.c"
#pragma GCC diagnostic pop
