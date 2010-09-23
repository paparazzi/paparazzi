#ifndef GPS_HW_H
#define GPS_HW_H

#define GpsBuffer() 0
#define ReadGpsBuffer() {}

void gps_feed_values(double utm_north, double utm_east, double utm_alt, double gspeed, double course, double climb);

#endif /* GPS_HW_H */
