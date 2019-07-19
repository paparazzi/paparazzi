// NPS wrapper around mt9f002.c

#define front_camera mt9f002_front_camera // Do not overwrite NPS camera struct

#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "mt9f002.c"
#pragma GCC diagnostic pop
