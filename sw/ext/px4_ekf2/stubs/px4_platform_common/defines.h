/*
 * Minimal replacement of the PX4 platform defines for the EKF2 in Paparazzi
 * (only what the EKF2 and its libraries use).
 */
#pragma once

#define PX4_ERROR (-1)
#define PX4_OK 0

#ifdef __cplusplus
static inline constexpr bool PX4_ISFINITE(float x) { return __builtin_isfinite(x); }
static inline constexpr bool PX4_ISFINITE(double x) { return __builtin_isfinite(x); }
#else
#define PX4_ISFINITE(x) __builtin_isfinite(x)
#endif

#define M_E_F       2.71828183f
#define M_LOG2E_F   1.44269504f
#define M_LOG10E_F  0.43429448f
#define M_LN2_F     0.69314718f
#define M_LN10_F    2.30258509f
#define M_PI_F      3.14159265f
#define M_TWOPI_F   6.28318531f
#define M_PI_2_F    1.57079632f
#define M_PI_4_F    0.78539816f
#define M_3PI_4_F   2.35619449f
#define M_SQRTPI_F  1.77245385f
#define M_1_PI_F    0.31830989f
#define M_2_PI_F    0.63661977f
#define M_2_SQRTPI_F 1.12837917f
#define M_DEG_TO_RAD_F 0.0174532925f
#define M_RAD_TO_DEG_F 57.2957795f
#define M_SQRT2_F   1.41421356f
#define M_SQRT1_2_F 0.70710678f
#define M_SQRT3_F   1.73205081f

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884
#endif
#define M_DEG_TO_RAD 0.017453292519943295
#define M_RAD_TO_DEG 57.295779513082323
