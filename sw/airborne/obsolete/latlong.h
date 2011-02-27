#ifndef LATLONG_H
#define LATLONG_H

extern float latlong_utm_x, latlong_utm_y; /* m */
extern float latlong_lat, latlong_lon; /* rad */

/** Convert geographic coordinates in a given UTM zone */
void latlong_utm_of(float lat_rad, float lon_rad, uint8_t utm_zone);
void latlong_of_utm(float x, float y, uint8_t utm_zone);

#endif
