#ifndef LISA_OVERO_LINK_H
#define LISA_OVERO_LINK_H

#include <inttypes.h>

#include "fms/fms_autopilot_msg.h"

enum LisaOveroLinkStatus {IDLE, BUSY, DATA_AVAILABLE, LOST};

#define OVERO_LINK_TIMEOUT 10

struct LisaOveroLink {
  volatile uint8_t status;
  uint32_t msg_cnt;
  uint32_t crc_err_cnt;
  union {
    struct OVERO_LINK_MSG_UP msg;
    uint8_t array[sizeof(union AutopilotMessage)];
  } up;
  union {
    struct OVERO_LINK_MSG_DOWN msg;
    uint8_t array[sizeof(union AutopilotMessage)];
  } down;
  uint8_t timeout_cnt;
  /* flags used to reset hardware */
  uint8_t crc_error;
  uint8_t timeout;
};

extern struct LisaOveroLink overo_link;

extern void overo_link_init(void);
extern void overo_link_periodic(void);

/* implemented by underlying architecture code */
extern void overo_link_arch_init(void);
extern void overo_link_arch_prepare_next_transfert(void);

#ifndef SITL
#include "lisa_overo_link_arch.h"
#endif

#if 0  /* that doesn't work yet */
#define OveroLinkPeriodic(_timeout_handler) {				\
    if (overo_link.timeout_cnt < OVERO_LINK_TIMEOUT)			\
      overo_link.timeout_cnt++;						\
    else {								\
      if (overo_link.status != LOST && overo_link.status != DATA_AVAILABLE ) { \
	SPI_Cmd(SPI1, DISABLE);						\
	overo_link.status = LOST;					\
	LED_OFF(OVERO_LINK_LED_OK);					\
	LED_ON(OVERO_LINK_LED_KO);					\
	overo_link.timeout = TRUE;					\
	_timeout_handler();						\
      }									\
    }									\
  }
#else   /* this one does */
#define OveroLinkPeriodic(_timeout_handler) {				\
    if (overo_link.timeout_cnt < OVERO_LINK_TIMEOUT)			\
      overo_link.timeout_cnt++;						\
    else {								\
      __disable_irq();							\
      if (overo_link.status != LOST && overo_link.status != DATA_AVAILABLE ) { \
	overo_link.status = LOST;					\
	__enable_irq();							\
	LED_OFF(OVERO_LINK_LED_OK);					\
	LED_ON(OVERO_LINK_LED_KO);					\
	_timeout_handler();						\
      }									\
      __enable_irq();							\
    }									\
  }
#endif






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



