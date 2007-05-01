#ifndef LATLONG_H
#define LATLONG_H

extern float latlong_utm_x, latlong_utm_y;

void latlong_utm_of(float phi, float lambda, uint8_t utm_zone);

#endif
