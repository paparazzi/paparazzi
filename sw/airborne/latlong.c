/** c.f. sw/lib/ocaml/latlong.ml */

#include <inttypes.h>
#include <math.h>

/* Computation for the WGS84 geoid only */
#define E 0.08248325676
#define K0 0.9996
#define XS 500000.
#define N (K0*E)

static const float serie_coeff_proj_mercator[5] = {
  0.99832429842242842444, 
  0.00083632803657738403,
  0.00000075957783563707,
  0.00000000119563131778,
  0.00000000000241079916
};

struct geographic { float lat; float lon; };
struct utm { float x; float y; uint8_t zone; };

float isometric_latitude(float phi) {
  return log (tan (M_PI_4 + phi / 2.0)) - E / 2.0 * log((1.0 + E * sin(phi)) / (1.0 - E * sin(phi)));
}
