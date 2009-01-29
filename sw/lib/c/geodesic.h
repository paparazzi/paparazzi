#ifndef GEODESIC_H
#define GEODESIC_H


struct Pprz_double_utm {
  double east;
  double north;
  double alt;
  int zone;
};

struct Pprz_double_lla {
  double lat;
  double lon;
  double alt;
};



void lla_of_utm(struct Pprz_double_lla *lla, struct Pprz_double_utm *utm);


#endif /* GEODESIC_H */

