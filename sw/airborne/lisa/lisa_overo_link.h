#ifndef LISA_OVERO_LINK_H
#define LISA_OVERO_LINK_H

#include <inttypes.h>

#include "fms/fms_autopilot_msg.h"

enum LisaOveroLinkStatus {IDLE, BUSY, DATA_AVAILABLE, LOST};

#define OVERO_LINK_TIMEOUT 10

struct LisaOveroLink {
	volatile uint8_t status;
	union {
		union OVERO_LINK_MSG_UNION uni;
		uint8_t array[sizeof(union OVERO_LINK_MSG_UNION)];
	} msg_in;
	union {
		union OVERO_LINK_MSG_UNION uni;
		uint8_t array[sizeof(union OVERO_LINK_MSG_UNION)];
	} msg_out;
	uint8_t timeout;
};

extern struct LisaOveroLink overo_link;

extern void overo_link_init(void);
extern void overo_link_periodic(void);

/* implemented by underlying architecture code */
extern void overo_link_arch_init(void);
extern void overo_link_arch_prepare_next_transfert(void);


#define OveroLinkEvent(_data_received_handler) {	\
    if (overo_link.status == DATA_AVAILABLE) {		\
      overo_link.timeout = 0;				\
      LED_TOGGLE(OVERO_LINK_LED_OK);			\
      LED_OFF(OVERO_LINK_LED_KO);			\
      _data_received_handler();				\
      overo_link_arch_prepare_next_transfert();		\
      overo_link.status = IDLE;				\
    }						        \
  }

#define OveroLinkPeriodic(_timeout_handler) {		\
    if (overo_link.timeout < OVERO_LINK_TIMEOUT)	\
      overo_link.timeout++;				\
    else {						\
      if (overo_link.status != LOST) {			\
	overo_link.status = LOST;			\
	LED_OFF(OVERO_LINK_LED_OK);			\
	LED_ON(OVERO_LINK_LED_KO);			\
	_timeout_handler();				\
      }							\
    }							\
  }



/*
 *
 * Passing telemetry through Overo Link
 * 
 */

#ifdef USE_OVERO_LINK_TELEMETRY

#define OVERO_LINK_TELEMETRY_BUF_SIZE 16

extern uint8_t overo_link_telemetry_insert_idx;
extern uint8_t overo_link_telemetry_extract_idx;
extern uint8_t overo_link_telemetry_buf[OVERO_LINK_TELEMETRY_BUF_SIZE];

#define OveroLinkTelemetryCheckFreeSpace(_x) (TRUE)
#define OveroLinkTelemetryTransmit(_x) {				\
    uint16_t temp = (overo_link_telemetry_insert_idx + 1) % OVERO_LINK_TELEMETRY_BUF_SIZE; \
    if (temp != overo_link_telemetry_extract_idx) { /* we have room */	\
      overo_link_telemetry_buf[overo_link_telemetry_insert_idx] = _x;	\
      overo_link_telemetry_insert_idx = temp;				\
    }									\
  }
#define OveroLinkTelemetrySendMessage() {}

extern uint8_t overo_link_telemetry_get(char* buf, int len);

#endif /* USE_OVERO_LINK_TELEMETRY */

#endif /* LISA_OVERO_LINK_H */



