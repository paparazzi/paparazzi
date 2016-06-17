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
 * @file modules/loggers/sdlog_chibios/msg_queue.h
 *
 */
#pragma once

#include <ch.h>
#include <hal.h>
#include "modules/tlsf/tlsf_malloc.h"


#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
  MsgQueue_MAILBOX_FULL = -100,
  MsgQueue_MAILBOX_FAIL,
  MsgQueue_MAILBOX_TIMEOUT,
  MsgQueue_MAILBOX_NOT_EMPTY,
  MsgQueue_INVALID_PTR,
  MsgQueue_OK = 0
} MsgQueueStatus;

typedef enum { MsgQueue_REGULAR, MsgQueue_OUT_OF_BAND } MsgQueueUrgency;

typedef struct  MsgQueue MsgQueue;


/**
 * @brief initialise MsgQueue
 * @details init sdQueue objet, tie memory pool buffer, tie mailbox buffer
 *              initialize mailbox
 * @param[out]  que:    opaque object to be initialized
 * @param[in] heap:   reference to the tlsf heap object (needed if module should free memory)
 * @param[in] mb_buff:  internal buffer used by MailBox (see Chibios Doc)
 * @param[in] mb_size:  size of previous buffer (length of MailBox queue)
 */
void msgqueue_init(MsgQueue *que, tlsf_memory_heap_t *heap,
                   msg_t *mb_buf, const cnt_t mb_size);

/**
 * @brief test if queue is full
 * @param[in]   que:    pointer to opaque MsgQueue object
 * @return  true if full, false otherwise
 */
bool msgqueue_is_full(MsgQueue *que);

/**
 * @brief test if queue is empty
 * @param[in]   que:  pointer to opaque MsgQueue object
 * @return  true if empty, false otherwise
 */
bool msgqueue_is_empty(MsgQueue *que);


/**
 * @brief send a buffer previously allocated by msgqueue_malloc_before_send
 * @details deallocation is done by the caching thread which actually write data to sd card
 *    from the sender point of view, ptr should be considered invalid after beeing sent.
 *    Even if msgqueue_send fail, deallocation is done
 *    Non blocking, if queue is full, report error and immediately return
 *    usage : msg = tlsf_malloc(MSGQ_HEAP, msg, msgLen);
 *                            msgqueue_send (que, msg, msgLen, urgency);
 *
 * @param[in]   que:  pointer to opaque MsgQueue object
 * @param[in]   msg:  pointer to buffer given by msgqueue_malloc_before_send
 * @param[in]   msgLen: length of buffer (the same param that has been
 *    given to msgqueue_malloc_before_send)
 * @param[in]   urgency  regular=queued at end of fifo or out_of_band queued at start of fifo
 * @return  if > 0 : requested length
 *    if < 0 : error status (see errors at begining of this header file)
 */
int32_t msgqueue_send(MsgQueue *que, void *msg, const uint16_t msgLen,
                      const MsgQueueUrgency urgency);


/**
 * @brief send a buffer previously allocated by msgqueue_malloc_before_send
 * @details deallocation is done by the caching thread which actually write data to sd card
 *    from the sender point of view, ptr should be considered invalid after beeing sent.
 *    Even if msgqueue_send fail, deallocation is done
 *    usage : msg = tlsf_malloc(MSGQ_HEAP, msg, msgLen);
 *                            msgqueue_send (que, msg, msgLen, urgency);
 *
 * @param[in]   que:  pointer to opaque MsgQueue object
 * @param[in]   msg:  pointer to buffer given by msgqueue_malloc_before_send
 * @param[in]   msgLen: length of buffer (the same param that has been
 *    given to msgqueue_malloc_before_send)
 * @param[in]   urgency  regular=queued at end of fifo or out_of_band queued at start of fifo
 * @param[in]   timout : time to wait for MailBox avaibility (can be TIME_INFINITE or TIME_IMMEDIATE)
 * @return  if > 0 : requested length
 *    if < 0 : error status (see errors at begining of this header file)
 */
int32_t msgqueue_send_timeout(MsgQueue *que, void *msg, const uint16_t msgLen,
                              const MsgQueueUrgency urgency, const systime_t timout);


/**
 * @brief send a buffer *NOT* previously allocated
 * @details buffer is copied before beeing sent, buffer is still valid after this function
 *    returns. Less effective that zero copy alternatives which has been to be preferably
 *              used
 *    Non blocking, if queue is full, report error and immediately return
 * @param[in]   que:  pointer to opaque MsgQueue object
 * @param[in]   msg:  pointer to buffer
 * @param[in]   msgLen: length of buffer (the same param that has been
 *    given to msgqueue_malloc_before_send)
 * @param[in]   urgency  regular=queued at end of fifo or out_of_band queued at start of fifo
 * @return  if > 0 : requested length
 *    if < 0 : error status (see errors at begining of this header file)
 */
int32_t msgqueue_copy_send(MsgQueue *que, const void *msg, const uint16_t msgLen,
                           const MsgQueueUrgency urgency);


/**
 * @brief send a buffer *NOT* previously allocated
 * @details buffer is copied before beeing sent, buffer is still valid after this function
 *    returns. Less effective that zero copy alternatives which has been to be preferably
 *              used
 * @param[in]   que:  pointer to opaque MsgQueue object
 * @param[in]   msg:  pointer to buffer
 * @param[in]   msgLen: length of buffer (the same param that has been
 *    given to msgqueue_malloc_before_send)
 * @param[in]   urgency  regular=queued at end of fifo or out_of_band queued at start of fifo
 * @param[in]   timout : time to wait for MailBox avaibility (can be TIME_INFINITE or TIME_IMMEDIATE)
 * @return  if > 0 : requested length
 *    if < 0 : error status (see errors at begining of this header file)
 */
int32_t msgqueue_copy_send_timeout(MsgQueue *que, const void *msg, const uint16_t msgLen,
                                   const MsgQueueUrgency urgency, const systime_t timout);



/*
 * goal : (zero copy api)
 * get a message to be processed without copy (blocking)
 *
 * parameters : INOUT queue object
 *        OUT msg pointer
 * return value:
 *
 */

/**
 * @brief wait then receive message
 * @details this is zero copy api, blocking call
 *    usage : struct MyStruct *msg
 *                      msgqueue_pop (&que, void (void **) &msg);
 *                      use msg->myField etc etc
                        tlsf_free(MSGQ_HEAP, msg);
 * @param[in]   que:  pointer to opaque MsgQueue object
 * @param[out]  msgPtr: pointer to pointer to buffer
 * @return  if > 0 : length of received msg
 *    if < 0 : error status (see errors at begining of this header file)
 */
int32_t msgqueue_pop(MsgQueue *que, void **msgPtr);

/**
 * @brief receive message specifying timeout
 * @details this is zero copy api, blocking call
 *    usage : struct MyStruct *msg
 *                      msgqueue_pop (&que, void (void **) &msg);
 *                      use msg->myField etc etc
                        msgqueue_free_after_pop (&que, msg);
 * @param[in]   que:  pointer to opaque MsgQueue object
 * @param[out]  msgPtr: pointer to pointer to buffer
 * @param[in]   timout : time to wait for MailBox avaibility (can be TIME_INFINITE or TIME_IMMEDIATE)
 * @return  if > 0 : length of received msg
 *    if < 0 : error status (see errors at begining of this header file)
 */
int32_t msgqueue_pop_timeout(MsgQueue *que, void **msgPtr, const systime_t timout);


/*
  goal : (zero copy api)
  free memory of a precedently given message by PopChunk

  parameters : INOUT queue object
               IN    ChunkBufferRO pointer
 */



/**
 * @brief debug api
 * @details give ascii string corresponding to the given errno
 * @param[in]   errno : staus
 * @return  pointer to error string
 */
const char *msgqueue_strerror(const MsgQueueStatus errno);

/*
#                 _____    _____    _____  __      __   ___    _______   ______
#                |  __ \  |  __ \  |_   _| \ \    / /  / _ \  |__   __| |  ____|
#                | |__) | | |__) |   | |    \ \  / /  | |_| |    | |    | |__
#                |  ___/  |  _  /    | |     \ \/ /   |  _  |    | |    |  __|
#                | |      | | \ \   _| |_     \  /    | | | |    | |    | |____
#                |_|      |_|  \_\ |_____|     \/     |_| |_|    |_|    |______|
*/

struct MsgQueue {
  mailbox_t mb;
  tlsf_memory_heap_t *heap;
} ;


#ifdef __cplusplus
}
#endif

