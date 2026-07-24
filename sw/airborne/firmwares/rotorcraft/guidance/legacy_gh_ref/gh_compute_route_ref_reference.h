/*
 * Reference reconstruction of gh_compute_route_ref() (historically
 * firmwares/rotorcraft/guidance/guidance_h_ref.c), which no longer exists in
 * this tree -- the fixed-point "route reference angle" horizontal-guidance
 * model it belonged to was replaced by a float-vector reference model in
 * 57ff4b982 ("Ref model float", #2842). The current gh_saturate_speed()/
 * gh_saturate_accel() do a simple vector-magnitude clamp and have no
 * equivalent route-direction computation at all.
 *
 * The function body below is copied verbatim from the real pre-rewrite
 * source (paparazzi commit 603b40a51, immediately before 57ff4b982), the
 * function PR #717 (https://github.com/paparazzi/paparazzi/pull/717)
 * modified callers of. Uses the real pprz_algebra_int.h / pprz_trig_int.h
 * (both still present, unchanged, in this tree) rather than reimplemented
 * stand-ins.
 *
 * Not referenced by any build file. Not built.
 */

#pragma once

#include "math/pprz_algebra_int.h"
#include "math/pprz_trig_int.h"
#include <math.h>
#include <stdlib.h>

/* base as it existed before the float-model rewrite, copied verbatim. */
static inline void gh_compute_route_ref_reference(struct Int32Vect2 *ref_vector,
    int32_t *route_ref, int32_t *s_route_ref, int32_t *c_route_ref)
{
  float f_route_ref = atan2f(-ref_vector->y, -ref_vector->x);
  *route_ref = ANGLE_BFP_OF_REAL(f_route_ref);
  /* Compute North and East route components */
  PPRZ_ITRIG_SIN(*s_route_ref, *route_ref);
  PPRZ_ITRIG_COS(*c_route_ref, *route_ref);
  *c_route_ref = abs(*c_route_ref);
  *s_route_ref = abs(*s_route_ref);
}
