#include "nps_ivy.h"

#include <stdlib.h>
#include <stdio.h>
#include <Ivy/ivy.h>

#include "generated/airframe.h"
#include "math/pprz_algebra_double.h"
#include "subsystems/ins.h"
#include "subsystems/navigation/common_nav.h"

/* fixedwing specific Datalink Ivy functions */
void on_DL_MOVE_WP(IvyClientPtr app __attribute__ ((unused)),
                   void *user_data __attribute__ ((unused)),
                   int argc __attribute__ ((unused)), char *argv[]);

void nps_ivy_init(char* ivy_bus) {
  /* init ivy and bind some messages common to fw and rotorcraft */
  nps_ivy_common_init(ivy_bus);

  IvyBindMsg(on_DL_MOVE_WP, NULL, "^(\\S*) MOVE_WP (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
}

//TODO use datalink parsing from fixedwing instead of doing it here explicitly

#include "generated/settings.h"
#include "dl_protocol.h"
#include "subsystems/datalink/downlink.h"

#define MOfCm(_x) (((float)(_x))/100.)
#define MOfMM(_x) (((float)(_x))/1000.)

void on_DL_MOVE_WP(IvyClientPtr app __attribute__ ((unused)),
                   void *user_data __attribute__ ((unused)),
                   int argc __attribute__ ((unused)), char *argv[]) {

  if (atoi(argv[2]) == AC_ID) {
    uint8_t wp_id = atoi(argv[1]);
    float a = MOfMM(atoi(argv[5]));

    /* Computes from (lat, long) in the referenced UTM zone */
    struct LlaCoor_f lla;
    lla.lat = RadOfDeg((float)(atoi(argv[3]) / 1e7));
    lla.lon = RadOfDeg((float)(atoi(argv[4]) / 1e7));
    //printf("move wp id=%d lat=%f lon=%f alt=%f\n", wp_id, lla.lat, lla.lon, a);
    struct UtmCoor_f utm;
    utm.zone = nav_utm_zone0;
    utm_of_lla_f(&utm, &lla);
    nav_move_waypoint(wp_id, utm.east, utm.north, a);

    /* Waypoint range is limited. Computes the UTM pos back from the relative
       coordinates */
    utm.east = waypoints[wp_id].x + nav_utm_east0;
    utm.north = waypoints[wp_id].y + nav_utm_north0;
    DOWNLINK_SEND_WP_MOVED(DefaultChannel, DefaultDevice, &wp_id, &utm.east, &utm.north, &a, &nav_utm_zone0);
    printf("move wp id=%d east=%f north=%f zone=%i alt=%f\n", wp_id, utm.east, utm.north, utm.zone, a);
  }
}
