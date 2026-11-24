/*
 * Minimal replacement of the PX4 logging macros for the EKF2 in Paparazzi.
 * Messages are printed in simulation (SITL) and dropped on the autopilot.
 */
#pragma once

#include <inttypes.h>

#if defined(SITL)
#include <stdio.h>
#define PX4_INFO(fmt, ...)  printf("EKF2: " fmt "\n", ##__VA_ARGS__)
#define PX4_WARN(fmt, ...)  printf("EKF2 warning: " fmt "\n", ##__VA_ARGS__)
#define PX4_ERR(fmt, ...)   printf("EKF2 error: " fmt "\n", ##__VA_ARGS__)
#define PX4_INFO_RAW(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define PX4_INFO(fmt, ...)  do {} while (0)
#define PX4_WARN(fmt, ...)  do {} while (0)
#define PX4_ERR(fmt, ...)   do {} while (0)
#define PX4_INFO_RAW(fmt, ...) do {} while (0)
#endif
#define PX4_DEBUG(fmt, ...) do {} while (0)
