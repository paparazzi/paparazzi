

#include "protocol.h"

uint8_t mora_ck_a, mora_ck_b;


/** Receiving pprz messages */

// PPRZ parsing state machine
#define UNINIT      0
#define GOT_STX     1
#define GOT_LENGTH  2
#define GOT_MSGID   3
#define GOT_PAYLOAD 4
#define GOT_CRC1    5

struct mora_transport mora_protocol;

void parse_mora(struct mora_transport *t, uint8_t c)
{
//printf("%02X %d %d\n",c, t->status, t->error);

  switch (t->status) {
    case UNINIT:
      if (c == STX) {
        t->status++;
      }
      break;
    case GOT_STX:
      if (t->msg_received) {
        t->error++;
        goto error;
      }
      t->payload_len = c - 5; /* Counting STX, LENGTH and CRC1 and CRC2 */
      t->ck_a = t->ck_b = c;
      t->status++;
      t->payload_idx = 0;
      break;
    case GOT_LENGTH:
      t->msg_id = c;
      t->ck_a += c; t->ck_b += t->ck_a;
      t->status++;
      if (t->payload_len == 0) {
        t->status++;
      }
      break;
    case GOT_MSGID:
      t->payload[t->payload_idx] = c;
      t->ck_a += c; t->ck_b += t->ck_a;
      t->payload_idx++;
      if (t->payload_idx == t->payload_len) {
        t->status++;
      }
      break;
    case GOT_PAYLOAD:
      if (c != t->ck_a) {
        goto error;
      }
      t->status++;
      break;
    case GOT_CRC1:
      if (c != t->ck_b) {
        goto error;
      }
      t->msg_received = TRUE;
      goto restart;
    default:
      goto error;
  }
  return;
error:
  t->error++;
restart:
  t->status = UNINIT;
  return;
}
