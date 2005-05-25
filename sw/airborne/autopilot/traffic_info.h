#ifndef TI_H
#define TI_H

struct ac_info_ {float east, north, heading, alt;};

void 
set_the_other(uint8_t id, float utm_x, float utm_y, float heading, float alt);

struct ac_info_ *
get_the_other(uint8_t id);

#endif
