/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_SBP_H
#define LIBSWIFTNAV_SBP_H

#include "common.h"

/** \addtogroup sbp
 * \{ */

/** Return value indicating success. */
#define SBP_OK              0
/** Return value indicating message decoded and callback executed by sbp_process. */
#define SBP_OK_CALLBACK_EXECUTED 1
/** Return value indicating message decoded with no associated callback in sbp_process. */
#define SBP_OK_CALLBACK_UNDEFINED 2
/** Return value indicating an error with the callback (function defined). */
#define SBP_CALLBACK_ERROR -1
/** Return value indicating a CRC error. */
#define SBP_CRC_ERROR      -2
/** Return value indicating an error occured whilst sending an SBP message. */
#define SBP_SEND_ERROR     -3
/** Return value indicating an error occured because an argument was NULL. */
#define SBP_NULL_ERROR     -4


/** SBP callback function prototype definition. */
typedef void (*sbp_msg_callback_t)(u16 sender_id, u8 len, u8 msg[], void *context);

/** SBP callback node.
 * Forms a linked list of callbacks.
 * \note Must be statically allocated for use with sbp_register_callback().
 */
typedef struct sbp_msg_callbacks_node {
  u16 msg_type;                        /**< Message ID associated with callback. */
  sbp_msg_callback_t cb;               /**< Pointer to callback function. */
  void *context;                       /**< Pointer to a context */
  struct sbp_msg_callbacks_node *next; /**< Pointer to next node in list. */
} sbp_msg_callbacks_node_t;

/** State structure for processing SBP messages. */
typedef struct {
  enum {
    WAITING = 0,
    GET_TYPE,
    GET_SENDER,
    GET_LEN,
    GET_MSG,
    GET_CRC
  } state;
  u16 msg_type;
  u16 sender_id;
  u16 crc;
  u8 msg_len;
  u8 n_read;
  u8 msg_buff[256];
  void* io_context;
  sbp_msg_callbacks_node_t* sbp_msg_callbacks_head;
} sbp_state_t;

/** \} */

s8 sbp_register_callback(sbp_state_t* s, u16 msg_type, sbp_msg_callback_t cb, void* context,
                         sbp_msg_callbacks_node_t *node);
void sbp_clear_callbacks(sbp_state_t* s);
sbp_msg_callbacks_node_t* sbp_find_callback(sbp_state_t* s, u16 msg_type);
void sbp_state_init(sbp_state_t *s);
void sbp_state_set_io_context(sbp_state_t *s, void* context);
s8 sbp_process(sbp_state_t *s, u32 (*read)(u8 *buff, u32 n, void* context));
s8 sbp_send_message(sbp_state_t *s, u16 msg_type, u16 sender_id, u8 len, u8 *payload,
                    u32 (*write)(u8 *buff, u32 n, void* context));

#endif /* LIBSWIFTNAV_SBP_H */

