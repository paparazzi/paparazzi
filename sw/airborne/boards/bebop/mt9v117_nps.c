// NPS wrapper for mt9v117.c

#define bottom_camera mt9v117_bottom_camera // Do not overwrite NPS camera struct

#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "mt9v117.c"
#pragma GCC diagnostic pop
