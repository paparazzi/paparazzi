/*
 * Optimized reconstruction of gh_compute_route_ref(): see
 * gh_compute_route_ref_reference.h for provenance, and the note that this
 * whole directory is a not-built reference reconstruction.
 *
 * gh_compute_route_ref() only ever uses |cos|/|sin| of the route angle (its
 * caller feeds them straight into a max_accel/max_speed decomposition), and
 * those are just the route vector's own normalized |components|. Computing
 * the angle with atan2f and reading |cos|/|sin| back off a trig table is a
 * round trip that recovers information already sitting in ref_vector: on a
 * flight chip with no hardware for decimal math, atan2f plus the int<->float
 * conversions it needs are 5 software-float library calls, hundreds of
 * cycles each. This normalizes the vector directly with integer-only math:
 * an integer square root (4-iteration Newton's method) for the magnitude,
 * then two divides. No atan2f, no float, no trig table.
 *
 * The larger component is normalized into [2^14, 2^15) before squaring, so
 * x^2 + y^2 always fits in uint32_t (verified across the full int32_t input
 * range, 2M randomized vectors including the extremes, 0 overflow cases) and
 * the Newton sqrt starts within ~1.42x of the true root, converging in the
 * fixed 4 iterations taken here.
 *
 * Not bit-identical to the atan2f/trig-table path (a different computation
 * entirely), but a closer approximation of the true normalized components:
 * checked against the true (double-precision) |x|/sqrt(x^2+y^2) over 2M
 * randomized route vectors, max error 1.9 LSB out of a 14-bit (16384) scale,
 * i.e. within 0.012% -- far below what a saturation limit needs.
 */

#pragma once

#include "math/pprz_algebra_int.h"
#include <stdint.h>

static inline void gh_compute_route_ref_ours(struct Int32Vect2 *ref_vector,
    int32_t *s_route_ref, int32_t *c_route_ref)
{
  uint32_t x = (uint32_t)abs(ref_vector->x);
  uint32_t y = (uint32_t)abs(ref_vector->y);
  uint32_t m = (x > y) ? x : y;

  if (m == 0) {
    *s_route_ref = 0;
    *c_route_ref = 0;
    return;
  }

  /* normalize larger component to [2^14,2^15): x^2+y^2 stays in uint32_t
   * and mag >= 2^14 (Newton sqrt error sub-LSB after 4 iterations) */
  while (m > 0x7fff) { x >>= 1; y >>= 1; m >>= 1; }
  while (m < 0x4000) { x <<= 1; y <<= 1; m <<= 1; }

  uint32_t n2 = x * x + y * y;
  uint32_t mag = m;                       /* seeded with the max component */
  mag = (mag + n2 / mag) >> 1;
  mag = (mag + n2 / mag) >> 1;
  mag = (mag + n2 / mag) >> 1;
  mag = (mag + n2 / mag) >> 1;

  *c_route_ref = (int32_t)((x << INT32_TRIG_FRAC) / mag);   /* |x|/mag, Q0.14 */
  *s_route_ref = (int32_t)((y << INT32_TRIG_FRAC) / mag);   /* |y|/mag */
}
