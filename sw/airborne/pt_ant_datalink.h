#ifndef PT_ANT_DATALINK_H
#define PT_ANT_DATALINK_H

#include "std.h"

extern bool_t pt_ant_dl_msg_available;
#define PT_ANT_MSG_SIZE 128
extern uint8_t pt_ant_dl_buffer[PT_ANT_MSG_SIZE]  __attribute__ ((aligned));

#define DlEventCheckAndHandle() {		\
    if (PprzBuffer()) {				\
      ReadPprzBuffer();				\
      if (pprz_msg_received) {			\
	pprz_parse_payload();			\
	pprz_msg_received = FALSE;		\
      }						\
    }						\
    if (pt_ant_dl_msg_available) {		\
      pt_ant_dl_parse_msg();			\
      pt_ant_dl_msg_available = FALSE;		\
    }						\
  }

extern void pt_ant_dl_parse_msg(void);

#endif /* PT_ANT_DATALINK_H */
