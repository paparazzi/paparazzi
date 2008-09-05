#define DATALINK_C
#include "datalink.h"

#include "settings.h"
#include "downlink.h"
#include "messages.h"
#include "dl_protocol.h"
#include "uart.h"

#define IdOfMsg(x) (x[1])

void dl_parse_msg(void) {
  uint8_t msg_id = IdOfMsg(dl_buffer);
  switch (msg_id) {
  
  case  DL_PING: {
    DOWNLINK_SEND_PONG();
    break;
  }
    
  case DL_SETTING : {
    uint8_t i = DL_SETTING_index(dl_buffer);
    float var = DL_SETTING_value(dl_buffer);
    DlSetting(i, var);
    DOWNLINK_SEND_DL_VALUE(&i, &var);
    break;
  }

  }
}
