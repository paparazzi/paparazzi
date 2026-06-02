

#include "std.h"
#include "mcu_periph/uart.h"
#include "modules/core/abi.h"
#include "modules/core/abi_sender_ids.h"
#include "modules/decawave/trilateration.h"
#include "generated/airframe.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/** frame sync byte */
#define DW_STX 0xFE

/** Parsing states */
#define DW_WAIT_STX 0
#define DW_GET_DATA 1
#define DW_GET_CK 2
#define DW_NB_DATA 6

/** DW1000 positionning system structure */
struct DW1000_periph {
  struct uart_periph *dev;
  uint8_t buf[DW_NB_DATA];    ///< incoming data buffer
  uint8_t idx;                ///< buffer index
  uint8_t ck;                 ///< checksum
  uint8_t state;              ///< parser state
};

struct DW1000_periph dw1000_periph;


/** Utility function to get float from buffer */
static inline float float_from_buf(uint8_t* b) {
  float f;
  memcpy((uint8_t*)(&f), b, sizeof(float));
  return f;
}

/** Utility function to get uint16_t from buffer */
static inline uint16_t uint16_from_buf(uint8_t* b) {
  uint16_t u16;
  memcpy ((uint8_t*)(&u16), b, sizeof(uint16_t));
  return u16;
}

/** Utility function to fill anchor from buffer */
static void send_anchor_data(struct DW1000_periph *dw) {
  uint16_t dst_id = uint16_from_buf(dw->buf);
  uint16_t src_id = DW1000_TAG_ID;
  float raw_dist = float_from_buf(dw->buf + 2);
  uint32_t now_ts = get_sys_time_usec();

  AbiSendMsgUWB_RANGING(UWB_DW1000_ARDUINO_ID, now_ts, src_id, dst_id, raw_dist);
}

/** Data parsing function */
static void dw1000_arduino_parse(struct DW1000_periph *dw, uint8_t c)
{
  switch (dw->state) {

    case DW_WAIT_STX:
      /* Waiting Synchro */
      if (c == DW_STX) {
        dw->idx = 0;
        dw->ck = 0;
        dw->state = DW_GET_DATA;
      }
      break;

    case DW_GET_DATA:
      /* Read Bytes */
      dw->buf[dw->idx++] = c;
      dw->ck += c;
      if (dw->idx == DW_NB_DATA) {
        dw->state = DW_GET_CK;
      }
      break;

    case DW_GET_CK:
      /* Checksum */
      if (dw->ck == c) {
        send_anchor_data(dw);
      }
      dw->state = DW_WAIT_STX;
      break;

    default:
      dw->state = DW_WAIT_STX;
  }
}

void dw1000_arduino_periph_init(void)
{
  dw1000_periph.dev = &DW1000_ARDUINO_DEV;
}

void dw1000_arduino_periph_event(void)
{
  // Look for data on serial link and send to parser
  while (uart_char_available(dw1000_periph.dev)) {
    uint8_t ch = uart_getch(dw1000_periph.dev);
    dw1000_arduino_parse(&dw1000_periph, ch);
  }
}
