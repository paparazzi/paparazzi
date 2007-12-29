#ifndef SNAV_H
#define SNAV_H

#include "std.h"

extern float snav_desired_tow; /* time of week, s */

bool_t snav_init(uint8_t wp_a, float desired_course_rad, float desired_tow);
bool_t snav_circle1(void);
bool_t snav_route(void);
bool_t snav_circle2(void);
bool_t snav_on_time(void);

#endif // SNAV_H


