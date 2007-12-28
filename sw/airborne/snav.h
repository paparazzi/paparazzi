#ifndef SNAV_H
#define SNAV_H

#include "std.h"

bool_t snav_init(uint8_t wp_a, float desired_course_rad);
bool_t snav_circle1(void);
bool_t snav_route(void);
bool_t snav_circle2(void);

#endif // SNAV_H


