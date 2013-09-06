#include "nps_ivy.h"

#include <stdlib.h>
#include <Ivy/ivy.h>

#include "generated/airframe.h"
#include "math/pprz_algebra_double.h"
#include "nps_autopilot.h"
#include "nps_fdm.h"
#include "nps_sensors.h"
#include "subsystems/ins.h"
#include "firmwares/rotorcraft/navigation.h"

#ifdef RADIO_CONTROL_TYPE_DATALINK
#include "subsystems/radio_control.h"
#endif

#include NPS_SENSORS_PARAMS


/* rotorcraft specificDatalink Ivy functions */
static void on_DL_MOVE_WP(IvyClientPtr app __attribute__ ((unused)),
                          void *user_data __attribute__ ((unused)),
                          int argc __attribute__ ((unused)), char *argv[]);

void nps_ivy_init(char* ivy_bus) {
  /* init ivy and bind some messages common to fw and rotorcraft */
  nps_ivy_common_init(ivy_bus);

  IvyBindMsg(on_DL_MOVE_WP, NULL, "^(\\S*) MOVE_WP (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");

}

//TODO use datalink parsing from rotorcraft instead of doing it here explicitly

#include "generated/settings.h"
#include "dl_protocol.h"
#include "subsystems/datalink/downlink.h"
static void on_DL_MOVE_WP(IvyClientPtr app __attribute__ ((unused)),
                          void *user_data __attribute__ ((unused)),
                          int argc __attribute__ ((unused)), char *argv[]) {
  if (atoi(argv[2]) == AC_ID) {
    uint8_t wp_id = atoi(argv[1]);

    struct LlaCoor_i lla;
    struct EnuCoor_i enu;
    lla.lat = INT32_RAD_OF_DEG(atoi(argv[3]));
    lla.lon = INT32_RAD_OF_DEG(atoi(argv[4]));
    lla.alt = atoi(argv[5])*10 - ins_ltp_def.hmsl + ins_ltp_def.lla.alt;
    enu_of_lla_point_i(&enu,&ins_ltp_def,&lla);
    enu.x = POS_BFP_OF_REAL(enu.x)/100;
    enu.y = POS_BFP_OF_REAL(enu.y)/100;
    enu.z = POS_BFP_OF_REAL(enu.z)/100;
    VECT3_ASSIGN(waypoints[wp_id], enu.x, enu.y, enu.z);
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id, &enu.x, &enu.y, &enu.z);
    printf("move wp id=%d x=%d y=%d z=%d\n", wp_id, enu.x, enu.y, enu.z);
  }
}
