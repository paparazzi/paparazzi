#define DATALINK_C
#include "datalink.h"

#include "settings.h"
#include "downlink.h"
#include "messages.h"
#include "dl_protocol.h"
#include "uart.h"

#ifdef BOOZ2_FMS_TYPE
#include "booz2_fms.h"
#endif

#include "booz2_navigation.h"

#define IdOfMsg(x) (x[1])

void dl_parse_msg(void) {
  uint8_t msg_id = IdOfMsg(dl_buffer);
  switch (msg_id) {
  
  case  DL_PING:
    {
      DOWNLINK_SEND_PONG();
    }
    break;
    
  case DL_SETTING :
    {
      uint8_t i = DL_SETTING_index(dl_buffer);
      float var = DL_SETTING_value(dl_buffer);
      DlSetting(i, var);
      DOWNLINK_SEND_DL_VALUE(&i, &var);
    }
    break;

#if defined BOOZ2_FMS_TYPE && BOOZ2_FMS_TYPE == BOOZ2_FMS_TYPE_DATALINK
  case DL_BOOZ2_FMS_COMMAND :
    {
      if (DL_BOOZ2_FMS_COMMAND_ac_id(dl_buffer) != AC_ID) break;
      BOOZ_FMS_PARSE_DATALINK(dl_buffer);
    }
    break;
#endif

  case DL_BLOCK :
    {
      if (DL_BLOCK_ac_id(dl_buffer) != AC_ID) break;
      nav_goto_block(DL_BLOCK_block_id(dl_buffer));
    }
    break;

  case DL_MOVE_WP :
    {
      uint8_t ac_id = DL_MOVE_WP_ac_id(dl_buffer);
      if (ac_id != AC_ID) break;
      uint8_t wp_id = DL_MOVE_WP_wp_id(dl_buffer);
      int32_t lat = DL_MOVE_WP_lat(dl_buffer);
      int32_t lon = DL_MOVE_WP_lon(dl_buffer);
      int32_t alt = DL_MOVE_WP_alt(dl_buffer);
      //PPRZ_INT32_LLA_ASSIGN(waypoints[wp_id], lat, lon, alt);
      DOWNLINK_SEND_WP_MOVED_LLA(&wp_id, &lat, &lon, &alt);
    }
    break;

  }
}
