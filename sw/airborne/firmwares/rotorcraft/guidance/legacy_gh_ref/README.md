# legacy_gh_ref (historical reference, not built)

This directory is **not referenced by any build file and is not compiled**. `gh_compute_route_ref()`,
the function [PR #717](https://github.com/paparazzi/paparazzi/pull/717) modified callers of, no longer
exists in this tree: the fixed-point "route reference angle" horizontal-guidance model it belonged to
was replaced by a float-vector reference model in `57ff4b982` ("Ref model float", #2842).
`gh_saturate_speed()`/`gh_saturate_accel()`, its current successors in `guidance_h_ref.c`, do a plain
vector-magnitude clamp and have no route-direction computation left to optimize.

`gh_compute_route_ref_reference.h`'s `gh_compute_route_ref_reference()` is copied verbatim from the
real pre-rewrite source (paparazzi commit `603b40a51`, immediately before `57ff4b982`), using the real
`pprz_algebra_int.h`/`pprz_trig_int.h` (both still present, unchanged, in this tree) rather than
reimplemented stand-ins.

`gh_compute_route_ref_ours.h` replaces the atan2f-then-trig-table round trip with a direct integer
vector normalization (Newton's-method integer square root + two divides). Checked against the true
(double-precision) normalized components over 2,000,000 randomized route vectors spanning the full
`int32_t` range: max error 1.9 LSB out of a 14-bit scale (0.012%), and 0 overflow cases in the
intermediate `x^2 + y^2` sum the normalize-into-range step is meant to guard against. Not
bit-identical to the atan2f path (a different computation), but the closer approximation of the two
to the true value.
