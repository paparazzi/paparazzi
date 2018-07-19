/*
 * Copyright (C) 2016 Gautier Hattenberger, Alexandre Bustico
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/*
 * @file modules/loggers/sdlog_chibios/msg_queue.c
 *
 */
#include "modules/loggers/sdlog_chibios/msg_queue.h"
#include "modules/tlsf/tlsf_malloc.h"
#include <string.h>

#define MSGQ_HEAP HEAP_CCM


typedef union {
  struct {
    uint16_t ptrOfst;
    uint16_t len;
  };
  msg_t msg_ptr_len;
} MsgPtrLen;


void    msgqueue_init(MsgQueue *que, tlsf_memory_heap_t *heap,
                      msg_t *mb_buf, const cnt_t mb_size)
{
  chMBObjectInit(&que->mb, mb_buf, mb_size);
  memset(mb_buf, 0, mb_size * sizeof(msg_t));
  que->heap = heap;
}

bool    msgqueue_is_full(MsgQueue *que)
{
  chSysLock();
  const bool queue_full = chMBGetFreeCountI(&que->mb) <= 0;
  chSysUnlock();

  return queue_full;
}

bool    msgqueue_is_empty(MsgQueue *que)
{
  chSysLock();
  bool queue_empty = (chMBGetUsedCountI(&que->mb) <= 0);
  chSysUnlock();

  return queue_empty;
}


int32_t   msgqueue_send(MsgQueue *que, void *msg, const uint16_t msgLen,
                        const MsgQueueUrgency urgency)
{
  return msgqueue_send_timeout(que, msg, msgLen, urgency, TIME_IMMEDIATE);
}


int32_t   msgqueue_send_timeout(MsgQueue *que, void *msg, const uint16_t msgLen,
                                const MsgQueueUrgency urgency, const systime_t timout)
{
  // this code prevents compiler warning about a missing initializer of mpl
  MsgPtrLen m;
  m.len = msgLen;
  m.ptrOfst = (uint32_t) msg - (uint32_t) tlsf_get_heap_addr(&MSGQ_HEAP);
  const MsgPtrLen mpl = m;

#if CH_DBG_ENABLE_CHECKS
  if (((uint32_t) msg < (uint32_t) tlsf_get_heap_addr(&MSGQ_HEAP)) ||
      ((uint32_t) msg >= ((uint32_t) tlsf_get_heap_addr(&MSGQ_HEAP) + 65535))) {
#if CH_DBG_ENABLE_ASSERTS
    chSysHalt("MsgQueue_INVALID_PTR");
#else
    return MsgQueue_INVALID_PTR;
#endif
  }
#endif

  if (urgency == MsgQueue_REGULAR) {
    if (chMBPostTimeout(&que->mb, mpl.msg_ptr_len, timout) != MSG_OK) {
      goto fail;
    }
  } else {
    if (chMBPostAheadTimeout(&que->mb, mpl.msg_ptr_len, timout) != MSG_OK) {
      goto fail;
    }
  }
  return msgLen;

fail:

  tlsf_free_r(que->heap, msg);

  return  MsgQueue_MAILBOX_FULL;
}

int32_t   msgqueue_copy_send_timeout(MsgQueue *que, const void *msg, const uint16_t msgLen,
                                     const MsgQueueUrgency urgency, const systime_t timout)
{
  void *dst = tlsf_malloc_r(&MSGQ_HEAP, msgLen);

  if (dst == NULL) {
    return MsgQueue_MAILBOX_FULL;
  }

  memcpy(dst, msg, msgLen);
  return msgqueue_send_timeout(que, dst, msgLen, urgency, timout);
}

int32_t   msgqueue_copy_send(MsgQueue *que, const void *msg, const uint16_t msgLen,
                             const MsgQueueUrgency urgency)
{
  return msgqueue_copy_send_timeout(que, msg, msgLen, urgency, TIME_IMMEDIATE);
}


int32_t msgqueue_pop(MsgQueue *que, void **msgPtr)
{
  return msgqueue_pop_timeout(que, msgPtr, TIME_INFINITE);
}

int32_t msgqueue_pop_timeout(MsgQueue *que, void **msgPtr, const systime_t timout)
{
  MsgPtrLen mpl = {.ptrOfst = 0, .len = 0};

  msg_t status;
  do  {
    status = chMBFetchTimeout(&que->mb, (msg_t *) &mpl.msg_ptr_len, timout);
  } while (status == MSG_RESET);

  if (status != MSG_OK) {
    return MsgQueue_MAILBOX_TIMEOUT;
  }

  *msgPtr = (void *)(mpl.ptrOfst + (uint32_t) tlsf_get_heap_addr(&MSGQ_HEAP));
  return mpl.len;
}




static const char *_strerror[] = {"MsgQueue_MAILBOX_FULL",
                                  "MsgQueue_MAILBOX_FAIL",
                                  "MsgQueue_MAILBOX_TIMEOUT",
                                  "MsgQueue_MAILBOX_NOT_EMPTY",
                                  "MsgQueue_OK",
                                  "MsgQueue_INVALID_PTR",
                                  "MsgQueue_INVALID_ERRNO"
                                 };

const char     *msgqueue_strerror(const MsgQueueStatus _errno)
{
  size_t indice = _errno - MsgQueue_MAILBOX_FULL;
  const size_t arrayLen = sizeof(_strerror) / sizeof(_strerror[0]);
  indice = (indice < arrayLen) ? indice : arrayLen - 1;
  return _strerror[indice];
}




