/** c.f. sw/lib/ocaml/latlong.ml */

#include <inttypes.h>
#include <math.h>
#include "std.h"
#include "latlong.h"
#include "led.h"

float latlong_utm_x, latlong_utm_y; /* m */
float latlong_lat, latlong_lon; /* rad */

/* Computation for the WGS84 geoid only */
#define E 0.08181919106
#define K0 0.9996
#define XS 500000.
#define YS 0.
#define A 6378137.0
#define N (K0*A)

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

struct complex { float re; float im; };
#define CScal(k, z) { z.re *= k;  z.im *= k; }
#define CAdd(z1, z2) { z2.re += z1.re;  z2.im += z1.im; }
#define CSub(z1, z2) { z2.re -= z1.re;  z2.im -= z1.im; }
#define CI(z) { float tmp = z.re; z.re = - z.im; z.im = tmp; }
#define CExp(z) { float e = exp(z.re); z.re = e*cos(z.im); z.im = e*sin(z.im); }
/* Expanded #define CSin(z) { CI(z); struct complex _z = {-z.re, -z.im}; CExp(z); CExp(_z); CSub(_z, z); CScal(-0.5, z); CI(z); } */

#define CSin(z) { CI(z); struct complex _z = {-z.re, -z.im}; float e = exp(z.re); float cos_z_im = cos(z.im); z.re = e*cos_z_im; float sin_z_im = sin(z.im); z.im = e*sin_z_im; _z.re = cos_z_im/e; _z.im = -sin_z_im/e; CSub(_z, z); CScal(-0.5, z); CI(z); }

#define CPrint(z) printf("%.6f+%.6fi\n", z.re, z.im)


static inline float isometric_latitude(float phi, float e) {
  return log (tan (M_PI_4 + phi / 2.0)) - e / 2.0 * log((1.0 + e * sin(phi)) / (1.0 - e * sin(phi)));
}

static inline float isometric_latitude0(float phi) {
  return log (tan (M_PI_4 + phi / 2.0));
}

static inline float inverse_isometric_latitude(float lat, float e, float epsilon) {
  float exp_l = exp(lat);
  float phi0 = 2 * atan(exp_l) - M_PI_2;
  float phi_;
  uint8_t bound = 3; /* To be sure to return */

  do {
    phi_ = phi0;
    float sin_phi = e * sin(phi_);
    phi0 = 2 * atan (pow((1 + sin_phi) / (1. - sin_phi), e/2.) * exp_l) - M_PI_2;
    bound--;
  } while (bound && fabs(phi_ - phi0) > epsilon);
  return phi0;
}

/** use on ARM7 at 60MHz, i.e. percentage of cputime at 4Hz
    specialize CSin: 951us i.e.
    One CSin: 120us
    init:
    iso_lat: 155, 172
    for loop: 258
*/

#define LambdaOfUtmZone(utm_zone) RadOfDeg((utm_zone-1)*6-180+3)

void latlong_utm_of(float phi, float lambda, uint8_t utm_zone) {
  float lambda_c = LambdaOfUtmZone(utm_zone);
  float ll = isometric_latitude(phi , E);
  float dl = lambda - lambda_c;
  float phi_ = asin(sin(dl) / cosh(ll));
  float ll_ = isometric_latitude0(phi_);
  float lambda_ = atan(sinh(ll) / cos(dl));
  struct complex z_ = { lambda_,  ll_ };
  CScal(serie_coeff_proj_mercator[0], z_);
  uint8_t k;
  for(k = 1; k < 3; k++) {
    struct complex z = { lambda_,  ll_ };
    CScal(2*k, z);
    CSin(z);
    CScal(serie_coeff_proj_mercator[k], z);
    CAdd(z, z_);
  }
  CScal(N, z_);
  latlong_utm_x = XS + z_.im;
  latlong_utm_y = z_.re;
}

void latlong_of_utm(float x, float y, uint8_t utm_zone) {
  float lambda_c = LambdaOfUtmZone(utm_zone);
  float scale = 1 / N / serie_coeff_proj_mercator[0];
  float real = (y - YS) * scale;
  float img = (x - XS) * scale;
  struct complex z = { real, img };

  uint8_t k;
  for(k = 1; k < 2; k++) {
    struct complex z_ = { real, img };
    CScal(2*k, z_);
    CSin(z_);
    CScal(serie_coeff_proj_mercator_inverse[k], z_);
    CSub(z_, z);
  }
  float ll = z.re;
  float lls = z.im;
  latlong_lon = lambda_c + atan (sinh(lls) / cos(ll));
  float phi_ = asin (sin(ll) / cosh(lls));
  ll = isometric_latitude0(phi_);
  latlong_lat = inverse_isometric_latitude(ll, E, 1e-8);
}


#ifdef TEST_LATLONG
#include <stdio.h>

int main () {
  struct complex z = { 1., 1.};
  CExp(z);
  CPrint(z);

  CSin(z);
  CPrint(z);

  latlong_utm_of(0.75, 0.02, 31);
  printf("utm_x=%.2f utm_y=%.2f {utm_x = 348805.714503056777; utm_y = 4759354.89376700297; utm_zone = 31}\n", latlong_utm_x, latlong_utm_y);

  latlong_utm_of(0.5, -1.5707, 16);
  printf("utm_x=%.2f utm_y=%.2f {utm_x = 207282.313014950836; utm_y = 3172645.2213267833; utm_zone = 16}\n", latlong_utm_x, latlong_utm_y);

  latlong_of_utm(348805.71, 4759354.89, 31);
  printf("lat=%.8f lon=%.8f posn_lat = 0.749999999392454875; posn_long = 0.019999999054505127}\n", latlong_lat, latlong_lon);

  return 0;
}
#endif
