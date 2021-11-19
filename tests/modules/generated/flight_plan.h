#ifndef GENERATED_FLIGHT_PLAN_H
#define GENERATED_FLIGHT_PLAN_H

#include "std.h"
#include "generated/modules.h"
#include "modules/core/abi.h"
#include "autopilot.h"

#define NAV_DEFAULT_ALT 260 /* nominal altitude of the flight plan */
#define NAV_UTM_EAST0 360285
#define NAV_UTM_NORTH0 4813595
#define NAV_UTM_ZONE0 31
#define NAV_LAT0 434622300 /* 1e7deg */
#define NAV_LON0 12728900 /* 1e7deg */
#define NAV_ALT0 185000 /* mm above msl */
#define NAV_MSL0 51850 /* mm, EGM96 geoid-height (msl) over ellipsoid */
#define QFU 0.0
#define WP_dummy 0
#define WP_HOME 1
#define WAYPOINTS_UTM { \
 {42.0, 42.0, 260},\
 {0.0, 0.0, 260},\
};
#define WAYPOINTS_ENU { \
 {41.13, 42.87, 75.00}, /* ENU in meters  */ \
 {-0.00, -0.00, 75.00}, /* ENU in meters  */ \
};
#define WAYPOINTS_LLA { \
 {.lat=434626158, .lon=12733981, .alt=260000}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=51.85m) */ \
 {.lat=434622299, .lon=12728900, .alt=260000}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=51.85m) */ \
};
#define WAYPOINTS_LLA_WGS84 { \
 {.lat=434626158, .lon=12733981, .alt=311850}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \
 {.lat=434622299, .lon=12728900, .alt=311850}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \
};
#define WAYPOINTS_GLOBAL { \
 FALSE, \
 FALSE, \
};
#define NB_WAYPOINT 2
#define FP_BLOCKS { \
 "Wait GPS" , \
 "HOME" , \
}
#define NB_BLOCK 2

#define GROUND_ALT 185.
#define GROUND_ALT_CM 18500
#define SECURITY_HEIGHT 25.
#define SECURITY_ALT 210.
#define HOME_MODE_HEIGHT 25.
#define MAX_DIST_FROM_HOME 1500.

#ifdef NAV_C

static inline void auto_nav_init(void) {}


static inline void auto_nav(void) {}

#endif

#endif // GENERATED_FLIGHT_PLAN_H
