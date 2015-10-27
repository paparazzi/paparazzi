#include "nps_ivy.h"

#include <stdlib.h>
#include <stdio.h>
#include <Ivy/ivy.h>

#include "generated/airframe.h"
#include "math/pprz_algebra_double.h"
#include "nps_autopilot.h"
#include "nps_fdm.h"
#include "nps_sensors.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"

#ifdef RADIO_CONTROL_TYPE_DATALINK
#include "subsystems/radio_control.h"
#endif

#include NPS_SENSORS_PARAMS


/* rotorcraft specificDatalink Ivy functions */
static void on_DL_MOVE_WP(IvyClientPtr app __attribute__((unused)),
                          void *user_data __attribute__((unused)),
                          int argc __attribute__((unused)), char *argv[]);

void nps_ivy_init(char *ivy_bus)
{
  /* init ivy and bind some messages common to fw and rotorcraft */
  nps_ivy_common_init(ivy_bus);
  IvyBindMsg(on_DL_MOVE_WP, NULL, "^(\\S*) MOVE_WP (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");

#if USE_MISSION_COMMANDS_IN_NPS
  nps_ivy_mission_commands_init();
#endif
}


#include "generated/settings.h"
#include "dl_protocol.h"
#include "subsystems/datalink/downlink.h"
static void on_DL_MOVE_WP(IvyClientPtr app __attribute__((unused)),
                          void *user_data __attribute__((unused)),
                          int argc __attribute__((unused)), char *argv[])
{
  if (!autopilot.datalink_enabled) {
    return;
  }

  if (atoi(argv[2]) == AC_ID) {
    uint8_t wp_id = atoi(argv[1]);

    struct LlaCoor_i lla;
    lla.lat = atoi(argv[3]);
    lla.lon = atoi(argv[4]);
    /* WP_alt from message is alt above MSL in mm
     * lla.alt is above ellipsoid in mm
     */
    lla.alt = atoi(argv[5]) - state.ned_origin_i.hmsl + state.ned_origin_i.lla.alt;
    waypoint_move_lla(wp_id, &lla);
    printf("move wp id=%d x=% .4f, y=% .4f, z=% .4f\n", wp_id,
           WaypointX(wp_id), WaypointY(wp_id), WaypointAlt(wp_id));
  }
}
