

/* Computation for the WGS84 geoid only */
#define E 0.08181919106
#define K0 0.9996
#define DELTA_EAST  500000.
#define DELTA_NORTH 0.
#define A 6378137.0
#define N (K0*A)

#define LambdaOfUtmZone(utm_zone) RadOfDeg((utm_zone-1)*6-180+3)

static const float serie_coeff_proj_mercator[5] = {
  0.99832429842242842444,
  0.00083632803657738403,
  0.00000075957783563707,
  0.00000000119563131778,
  0.00000000000241079916
};

static const float serie_coeff_proj_mercator_inverse[5] = {
  0.998324298422428424,
  0.000837732168742475825,
  5.90586914811817062e-08,
  1.6734091890305064e-10,
  2.13883575853313883e-13
};
