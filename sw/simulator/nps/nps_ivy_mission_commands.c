#include "nps_ivy.h"

#include <stdlib.h>
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

#define MSG_SIZE 128
extern uint8_t dl_buffer[MSG_SIZE];

/* rotorcraft specificDatalink Ivy functions */
static void on_DL_MISSION_GOTO_WP(IvyClientPtr app __attribute__ ((unused)),
                          void *user_data __attribute__ ((unused)),
                          int argc __attribute__ ((unused)), char *argv[]);


void nps_ivy_mission_commands_init(void) {
 
    IvyBindMsg(on_DL_MISSION_GOTO_WP, NULL, "^(\\S*) MISSION_GOTO_WP (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
 
}

#include "generated/settings.h"
#include "dl_protocol.h"
#include "subsystems/datalink/downlink.h"
static void on_DL_MISSION_GOTO_WP(IvyClientPtr app __attribute__ ((unused)),
                          void *user_data __attribute__ ((unused)),
                          int argc __attribute__ ((unused)), char *argv[]) {
  uint8_t i = 0;  
  float dummy;
  dl_buffer[2] = (float)(atoi(argv[1]));
  dl_buffer[3] = (float)(atoi(argv[2]));
  
  for(i=1; i<5 ; i++){
    dummy = (float)(atoi(argv[2+i]));
    memcpy(&dl_buffer[i*4], &dummy, 4);
    //dl_buffer[i+4] = 5;
  }
  
  mission_parse_GOTO_WP();
}
