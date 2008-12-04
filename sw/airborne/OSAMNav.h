#ifndef OSAMNav_H
#define OSAMNav_H

#include "std.h"
#include "nav.h"
#include "estimator.h"
#include "autopilot.h"

extern bool_t FlowerNav(void);
extern bool_t InitializeFlower(uint8_t CenterWP, uint8_t EdgeWP);

extern bool_t InitializeBungeeTakeoff(uint8_t BungeeWP);
extern bool_t BungeeTakeoff(void);

#endif
