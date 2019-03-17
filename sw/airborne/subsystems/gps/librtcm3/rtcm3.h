/*
 * Copyright (C) 2016 Wilco Vlenterie, Anand Sundaresan.
 * Contact: Anand Sundaresan <nomail@donotmailme.com>
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBRTCM3_RTCM3_H
#define LIBRTCM3_RTCM3_H

/* Macros for RTCM message processing */
#define READ_RESERVED   1
#define READ_LENGTH     2
#define READ_MESSAGE    3
#define READ_CHECKSUM   4

#define RTCM3_PREAMBLE 0xD3
#define RTCM3_MSG_4072 0x72
#define RTCM3_MSG_1005 0x69
#define RTCM3_MSG_1077 0xB1
#define RTCM3_MSG_1087 0xBB
#define RTCM3_MSG_1097 0xC5
#define RTCM3_MSG_1127 0x7F
#define RTCM3_MSG_1230 0xE6

/* Macros for UBX message processing */

#define UNINIT          0
#define GOT_SYNC1       1
#define GOT_SYNC2       2
#define GOT_CLASS       3
#define GOT_ID          4
#define GOT_LEN1        5
#define GOT_LEN2        6
#define GOT_PAYLOAD     7
#define GOT_CHECKSUM1   8

#define UBX_PREAMBLE1       0xB5                // Sync 1
#define UBX_PREAMBLE2       0x62                // Sync 2
#define UBX_NAV_SVIN        0x3B
#define GPS_UBX_MAX_PAYLOAD 255

#define GPS_UBX_ERR_NONE         0
#define GPS_UBX_ERR_OVERRUN      1
#define GPS_UBX_ERR_MSG_TOO_LONG 2
#define GPS_UBX_ERR_CHECKSUM     3
#define GPS_UBX_ERR_UNEXPECTED   4
#define GPS_UBX_ERR_OUT_OF_SYNC  5

#define NO_CLASS   0
#define RTCM_CLASS 1
#define UBX_CLASS  2

#include <errno.h>
#include "common.h"
#include "CRC24Q.h"

/** Return value indicating success. */
#define RTCM_OK              0
/** Return value indicating message decoded and callback executed by rtcm3_process. */
#define RTCM_OK_CALLBACK_EXECUTED 1
/** Return value indicating message decoded with no associated callback in rtcm3_process. */
#define RTCM_OK_CALLBACK_UNDEFINED 2
/** Return value indicating an error with the callback (function defined). */
#define RTCM_CALLBACK_ERROR -1
/** Return value indicating a CRC error. */
#define RTCM_CRC_ERROR      -2
/** Return value indicating an error occured whilst sending an RTCM3 message. */
#define RTCM_SEND_ERROR     -3
/** Return value indicating an error occured because an argument was NULL. */
#define RTCM_NULL_ERROR     -4

/** RTCM3 callback function prototype definition. */

typedef void (*rtcm3_msg_callback_t)(u8 len, u8 msg[]);

/** RTCM3 callback node.
 * Forms a linked list of callbacks.
 * \note Must be statically allocated for use with rtcm3_register_callback().
 */
typedef struct rtcm3_msg_callbacks_node {
  u16 msg_type;                        /**< Message ID associated with callback. */
  rtcm3_msg_callback_t cb;               /**< Pointer to callback function. */
  //void *context;                       /**< Pointer to a context */
  struct rtcm3_msg_callbacks_node *next; /**< Pointer to next node in list. */
} rtcm3_msg_callbacks_node_t;

/* structure for processing RTCM and UBX messages */
typedef struct {
  // For RTCM processing
  u8 state;
  u16 msg_type;
  u8 msg_class;
  u16 crc;
  u16 msg_len;
  u8 n_read;
  u8 msg_buff[1024 + 6 + 1];
  rtcm3_msg_callbacks_node_t *rtcm3_msg_callbacks_head;
  u8 status;
  u8 ck_a, ck_b;
  u8 error_cnt;
  u8 error_last;
} msg_state_t;


/* Function prototypes */

s8                          rtcm3_register_callback(msg_state_t *s, u16 msg_type, rtcm3_msg_callback_t cb,
    rtcm3_msg_callbacks_node_t *node);
void                        rtcm3_clear_callbacks(msg_state_t *s);
rtcm3_msg_callbacks_node_t *rtcm3_find_callback(msg_state_t *s, u16 msg_type);
void                        msg_state_init(msg_state_t *s);

s8                          rtcm3_process(msg_state_t *s, unsigned char buff);
s8              ubx_process(msg_state_t *s, unsigned char buff);
unsigned int                RTCMgetbitu(unsigned char *, int, int);
int                         RTCMgetbits(unsigned char *, int , int);
static double               RTCMgetbits_38(unsigned char *, int);


/* Global variables used for RTCM processing */
int rd_msg_len      = 0;
int rd_msg_len1     = 0;
int byteIndex       = 0;
int checksumCounter = 0;
int rawIndex        = 0;

/** Register a callback for a message type.
 * Register a callback that is called when a message
 * with type msg_type is received.
 *
 * \param msg_type Message type associated with callback
 * \param cb       Pointer to message callback function
 * \param context  Pointer to context for callback function
 * \param node     Statically allocated #rtcm3_msg_callbacks_node_t struct
 * \return `RTCM_OK` (0) if successful, `RTCM_CALLBACK_ERROR` if callback was
 *         already registered for that message type.
 */
s8 rtcm3_register_callback(msg_state_t *s, u16 msg_type, rtcm3_msg_callback_t cb,
                           rtcm3_msg_callbacks_node_t *node)
{
  /* Check our callback function pointer isn't NULL. */
  if (cb == 0) {
    return RTCM_NULL_ERROR;
  }

  /* Check our callback node pointer isn't NULL. */
  if (node == 0) {
    return RTCM_NULL_ERROR;
  }

  /* Check if callback was already registered for this type. */
  if (rtcm3_find_callback(s, msg_type) != 0) {
    return RTCM_CALLBACK_ERROR;
  }

  /* Fill in our new rtcm3_msg_callback_node_t. */
  node->msg_type = msg_type;
  node->cb = cb;
  //node->context = context;
  /* The next pointer is set to NULL, i.e. this
   * will be the new end of the linked list.
   */
  node->next = 0;

  /* If our linked list is empty then just
   * add the new node to the start.
   */
  if (s->rtcm3_msg_callbacks_head == 0) {
    s->rtcm3_msg_callbacks_head = node;
    return RTCM_OK;
  }

  /* Find the tail of our linked list and
   * add our new node to the end.
   */
  rtcm3_msg_callbacks_node_t *p = s->rtcm3_msg_callbacks_head;
  while (p->next) {
    p = p->next;
  }

  p->next = node;

  return RTCM_OK;
}

/** Clear all registered callbacks.
 * This is probably only useful for testing but who knows!
 */
void rtcm3_clear_callbacks(msg_state_t *s)
{
  /* Reset the head of the callbacks list to NULL. */
  s->rtcm3_msg_callbacks_head = 0;
}

/** Find the callback function associated with a message type.
 * Searches through the list of registered callbacks to find the callback
 * associated with the passed message type.
 *
 * \param msg_type Message type to find callback for
 * \return Pointer to callback node (#rtcm3_msg_callbacks_node_t) or `NULL` if
 *         callback not found for that message type.
 */
rtcm3_msg_callbacks_node_t *rtcm3_find_callback(msg_state_t *s, u16 msg_type)
{
  /* If our list is empty, return NULL. */
  if (!s->rtcm3_msg_callbacks_head) {
    return 0;
  }

  /* Traverse the linked list and return the callback
   * function pointer if we find a node with a matching
   * message id.
   */
  rtcm3_msg_callbacks_node_t *p = s->rtcm3_msg_callbacks_head;
  do
    if (p->msg_type == msg_type) {
      return p;
    }

  while ((p = p->next));

  /* Didn't find a matching callback, return NULL. */
  return 0;
}

/** Initialize an #msg_state_t struct before use.
 * This resets the entire state, including all callbacks.
 * Remember to use this function to initialize the state before calling
 * rtcm3_process() for the first time.
 *
 * \param s State structure
 */
void msg_state_init(msg_state_t *s)
{
  s->state = UNINIT;
  s->msg_class = NO_CLASS;

  /* Set the IO context pointer, passed to read and write functions, to NULL. */
  // s->io_context = 0;

  /* Clear the callbacks, if any, currently in s */
  rtcm3_clear_callbacks(s);
}




/** Read and process RTCM3 messages.
 * Reads bytes from an input source using the provided `read` function, decodes
 * the RTCM3.
 *
 * When an RTCM3 message is successfully received then the list of callbacks is
 * searched for a callback corresponding to the received message type. If a
 * callback is found then it is called with the ID of the sender, the message
 * length and the message payload data buffer as arguments.
 *
 * \note rtcm3_process will always call `read` with n > 0
 *       (aka it will attempt to always read something)
 *
 * The supplied `read` function must have the prototype:
 *
 * ~~~
 * u32 read(u8 *buff, u32 n, void* context)
 * ~~~
 *
 * where `n` is the number of bytes requested and `buff` is the buffer into
 * which to write the received data, and `context` is the arbitrary pointer
 * set by `rtcm3_state_set_io_context`.
 * The function should return the number of
 * bytes successfully written into `buff` which may be between 0 and `n`
 * inclusive, but must never be greater than `n`.
 *
 * Note that `rtcm3_process` may not read all available bytes from the `read`
 * function so the caller should loop until all bytes available from the input
 * source have been consumed.
 *
 * \param s State structure
 * \param read Function pointer to a function that reads `n` bytes from the
 *             input source into `buff` and returns the number of bytes
 *             successfully read.
 * \return `RTCM_OK` (0) if successful but no complete message yet,
 *         `RTCM_OK_CALLBACK_EXECUTED` (1) if message decoded and callback executed,
 *         `RTCM_OK_CALLBACK_UNDEFINED` (2) if message decoded with no associated
 *         callback, and `RTCM_CRC_ERROR` (-2) if a CRC error
 *         has occurred. Thus can check for >0 to ensure good processing.
 */
s8 rtcm3_process(msg_state_t *s, unsigned char buff)
{
  /*  FAKE MESSAGE
    buff[s->n_read] = s->n_read;
    s->n_read++;
    int fakeMsgLen = 200;
    if(s->n_read == fakeMsgLen)
    {
      s->n_read = 0;
      rtcm3_msg_callbacks_node_t* node = rtcm3_find_callback(s, RTCM3_MSG_1077);
      (*node->cb)(s->sender_id, fakeMsgLen, buff, node->context);
      return RTCM_OK_CALLBACK_EXECUTED;
    }else{
      return RTCM_OK;
    }
   */
  if (s->n_read == (1024 + 6) && s->state != UNINIT) {
    // We have exceeded the maximum message length (10bit) + 3 opening and 3 closing bytes. And we are not at UNINIT, this is not a proper message, reset!
    s->state = UNINIT;
    s->msg_class = NO_CLASS;
  }
  // Suppose we get more bytes than requested, lets still process them all
#ifdef DEBUG_PRINT_PACKAGE
  printf("0x%x ", buff);
#endif
  if (s->state != UNINIT && s->msg_class == RTCM_CLASS) { s->msg_buff[s->n_read] = buff; }
  switch (s->state) {
    case UNINIT:
      s->n_read = 0;
      if (((int) buff) == RTCM3_PREAMBLE) {
        s->msg_class = RTCM_CLASS;
        s->state = READ_RESERVED;
        rawIndex        = 0;
        checksumCounter = 0;
        byteIndex       = 0;
        s->msg_buff[s->n_read] = buff;
      }
      break;
    case READ_RESERVED:
      rd_msg_len1 = ((int) buff) & 0b00000011;
      s->state    = READ_LENGTH;
      break;
    case READ_LENGTH:
      rd_msg_len  = (rd_msg_len1 << 8) + ((int) buff) ;
      s->state    = READ_MESSAGE;
      break;
    case READ_MESSAGE:
      if (byteIndex >= (rd_msg_len - 1)) { s->state = READ_CHECKSUM; }
      byteIndex++;
      break;
    case READ_CHECKSUM:
      checksumCounter++;
      if (checksumCounter == 3) {
#ifdef DEBUG_PRINT_PACKAGE
        printf("\n\n");
#endif
        s->state = UNINIT;
        s->n_read++;
        s->msg_len   = s->n_read;
        s->msg_class = NO_CLASS;

        // Check the checksum
        if(crc24q(s->msg_buff, s->n_read - 3) != RTCMgetbitu(s->msg_buff, (s->n_read - 3) * 8, 24))
          return RTCM_OK_CALLBACK_UNDEFINED;
        
        // Check what message type it is
        switch (RTCMgetbitu(s->msg_buff, 24 + 0, 12)) {
          case 4072: s->msg_type = RTCM3_MSG_4072; break;
          case 1005: s->msg_type = RTCM3_MSG_1005; break;
          case 1077: s->msg_type = RTCM3_MSG_1077; break;
          case 1087: s->msg_type = RTCM3_MSG_1087; break;
          case 1097: s->msg_type = RTCM3_MSG_1097; break;
          case 1127: s->msg_type = RTCM3_MSG_1127; break;
          case 1230: s->msg_type = RTCM3_MSG_1230; break;
          default  : printf("Unknown message type %d\n", RTCMgetbitu(s->msg_buff, 24 + 0, 12)); return RTCM_OK_CALLBACK_UNDEFINED;
        }
#ifdef NO_CALLBACK
        return RTCM_OK_CALLBACK_EXECUTED;
#else
        /* Message complete, process its callback. */
        rtcm3_msg_callbacks_node_t *node = rtcm3_find_callback(s, s->msg_type);
        if (node) {
          (*node->cb)(s->msg_len, s->msg_buff);
          return RTCM_OK_CALLBACK_EXECUTED;
        } else {
          return RTCM_OK_CALLBACK_UNDEFINED;
        }
#endif
      }
      break;
  }
  s->n_read++;
  return RTCM_OK;
}


// UBX message decoding

s8 ubx_process(msg_state_t *s, unsigned char buff)
{
  if (s->state < GOT_PAYLOAD && s->msg_class == UBX_CLASS) {
    s->ck_a += buff;
    s->ck_b += s->ck_a;
  }
  switch (s->state) {
    case UNINIT:
      if (buff == UBX_PREAMBLE1) {
        s->state++;
        s->msg_class = UBX_CLASS;
      } else {
        s->n_read = 0;
      }
      break;
    case GOT_SYNC1:
      if (buff != UBX_PREAMBLE2 && s->msg_class == UBX_CLASS) {
        s->error_last = GPS_UBX_ERR_OUT_OF_SYNC;
        goto error;
      }
      s->ck_a = 0;
      s->ck_b = 0;
      s->state ++;
      break;
    case GOT_SYNC2:
      //s->msg_class = buff;
      s->state++;
      break;
    case GOT_CLASS:
      s->msg_type = buff;
      s->state++;
      break;
    case GOT_ID:
      s->msg_len = buff;
      s->state++;
      break;
    case GOT_LEN1:
      s->msg_len |= (buff << 8);
      if (s->msg_len > GPS_UBX_MAX_PAYLOAD) {
        s->error_last = GPS_UBX_ERR_MSG_TOO_LONG;
        goto error;
      }
      s->n_read = 0;
      s->state++;
      break;
    case GOT_LEN2:
      s->msg_buff[s->n_read] = buff;
      s->n_read++;
      if (s->n_read >= s->msg_len) {
        s->state++;
      }
      break;
    case GOT_PAYLOAD:
      if (buff != s->ck_a) {
        s->error_last = GPS_UBX_ERR_CHECKSUM;
        goto error;
      }
      s->state++;
      break;
    case GOT_CHECKSUM1:
      if (buff != s->ck_b) {
        s->error_last = GPS_UBX_ERR_CHECKSUM;
        goto error;
      }
      s->msg_class = NO_CLASS;
      s->n_read = 0;
      s->state = UNINIT;
#ifdef NO_CALLBACK
      return RTCM_OK_CALLBACK_EXECUTED;
#else
      /* Message complete, process its callback. */
      rtcm3_msg_callbacks_node_t *node = rtcm3_find_callback(s, s->msg_type);
      if (node) {
        (*node->cb)(s->msg_len, s->msg_buff);
        return RTCM_OK_CALLBACK_EXECUTED;
      } else {
        return RTCM_OK_CALLBACK_UNDEFINED;
      }
#endif
      break;
    default:
      s->error_last = GPS_UBX_ERR_UNEXPECTED;
      goto error;
  }
  return RTCM_OK;
error:
  s->error_cnt++;
  s->state = UNINIT;
  s->msg_class = NO_CLASS;
  return s->error_last;
}

unsigned int RTCMgetbitu(unsigned char *buff, int pos, int lenb)
{
  unsigned int bits = 0;
  int i;
  for (i = pos; i < pos + lenb; i++) { bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u); }
  return bits;
}

int RTCMgetbits(unsigned char *buff, int pos, int lenb)
{
  unsigned int bits = RTCMgetbitu(buff, pos, lenb);
  if (lenb <= 0 || 32 <= lenb || !(bits & (1u << (lenb - 1)))) { return (int)bits; }
  return (int)(bits | (~0u << lenb)); /* extend sign */
}

static double RTCMgetbits_38(unsigned char *buff, int pos)
{
  return (double)RTCMgetbits(buff, pos, 32) * 64.0 + RTCMgetbitu(buff, pos + 32, 6);
}

#endif /* LIBRTCM3_RTCM3_H */
