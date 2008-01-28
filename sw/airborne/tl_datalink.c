#define DATALINK_C
#include "datalink.h"
#include "airframe.h"
#include "std.h"
#include "dl_protocol.h"
#include "settings.h"
#include "messages.h"
#include "tl_telemetry.h"

#define IdOfMsg(x) (x[1])

void dl_parse_msg(void) {
  uint8_t msg_id = IdOfMsg(dl_buffer);
  if (msg_id == DL_SETTING && DL_SETTING_ac_id(dl_buffer) == AC_ID) {
    uint8_t i = DL_SETTING_index(dl_buffer);
    float val = DL_SETTING_value(dl_buffer);
    DlSetting(i, val);
    DOWNLINK_SEND_DL_VALUE(&i, &val);
  }
}
