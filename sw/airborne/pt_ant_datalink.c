#include "pt_ant_datalink.h"

bool_t pt_ant_dl_msg_available;
uint8_t pt_ant_dl_buffer[PT_ANT_MSG_SIZE]  __attribute__ ((aligned));
#include "settings.h"
#include "downlink.h"
#include "messages.h"
#include "dl_protocol.h"
#include "uart.h"

#include "traffic_info.h"
#include "pt_ant_estimator.h"

#define IdOfMsg(x) (x[1])
#define MOfCm(_x) (((float)_x)/100.)

void pt_ant_dl_parse_msg(void) {
  uint8_t msg_id = IdOfMsg(pt_ant_dl_buffer);
  switch (msg_id) {
  
  case DL_SETTING : {
    uint8_t i = DL_SETTING_index(pt_ant_dl_buffer);
    float var = DL_SETTING_value(pt_ant_dl_buffer);
    DlSetting(i, var);
    DOWNLINK_SEND_DL_VALUE(&i, &var);
    break;
  }
  
  case DL_ACINFO: {
    LED_TOGGLE(1);
    uint8_t id = DL_ACINFO_ac_id(pt_ant_dl_buffer);
    struct ac_info_ ac;
    ac.east = MOfCm(DL_ACINFO_utm_east(pt_ant_dl_buffer));
    ac.north = MOfCm(DL_ACINFO_utm_north(pt_ant_dl_buffer));
    ac.course = MOfCm(DL_ACINFO_alt(pt_ant_dl_buffer));
    ac.alt = RadOfDeg(((float)DL_ACINFO_course(pt_ant_dl_buffer))/ 10.);
    ac.gspeed = MOfCm(DL_ACINFO_speed(pt_ant_dl_buffer));
    pt_ant_estimator_update_target(id, &ac);
    break;
  }
  
  }
}
