#define DATALINK_C
#include "datalink.h"

#include "settings.h"
#include "downlink.h"
#include "messages.h"
#include "dl_protocol.h"
#include "uart.h"

#include "traffic_info.h"
#include "pt_ant_estimator.h"

#define IdOfMsg(x) (x[1])
#define MOfCm(_x) (((float)_x)/100.)

void dl_parse_msg(void) {
  uint8_t msg_id = IdOfMsg(dl_buffer);
  switch (msg_id) {
  
  case DL_SETTING : {
    uint8_t i = DL_SETTING_index(dl_buffer);
    float var = DL_SETTING_value(dl_buffer);
    DlSetting(i, var);
    DOWNLINK_SEND_DL_VALUE(&i, &var);
    break;
  }
  
  case DL_ACINFO: {
    LED_TOGGLE(1);
    uint8_t id = DL_ACINFO_ac_id(dl_buffer);
    struct ac_info_ ac;
    ac.east = MOfCm(DL_ACINFO_utm_east(dl_buffer));
    ac.north = MOfCm(DL_ACINFO_utm_north(dl_buffer));
    ac.course = MOfCm(DL_ACINFO_alt(dl_buffer));
    ac.alt = RadOfDeg(((float)DL_ACINFO_course(dl_buffer))/ 10.);
    ac.gspeed = MOfCm(DL_ACINFO_speed(dl_buffer));
    pt_ant_estimator_update_target(id, &ac);
    break;
  }
  
  }
}
