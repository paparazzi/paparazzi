#ifndef ANEMOTAXIS_H
#define ANEMOTAXIS_H

#include "std.h"

extern bool nav_anemotaxis_downwind(uint8_t c, float radius);
extern bool nav_anemotaxis_init(uint8_t c);
extern bool nav_anemotaxis(uint8_t c, uint8_t c1, uint8_t c2, uint8_t plume);

#endif /** ANEMOTAXIS_H */
