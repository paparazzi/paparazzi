#ifndef SPIRAL_H
#define SPIRAL_H

#include "std.h"

extern bool_t SpiralNav(void);
extern bool_t InitializeSpiral(uint8_t CenterWP, uint8_t EdgeWP, float StartRad, float IncRad,
				float Segments, float ZKoord );

#endif

