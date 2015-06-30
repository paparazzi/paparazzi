#pragma once

/*

static declaration :
//VARLEN_MSGQUEUE_DECL(static, TRUE, vmlq, 16, 4, __attribute__ ((section(".ccmram"), aligned(8))));

arg1 : scope => nothing or static
arg2 : if ZeroCopy api is used to get message (varLenMsgQueuePopChunk and varLenMsgQueueFreeChunk)
       this arg has to be TRUE, elsewhere, it could be FALSE : better ram and speed optimisation
arg3 : name of VarLenMsgQueue object
arg4 : circularBuffer size in byte
arg5 : mailbox size in message (maximum number of pending message, even is circularBuffer is not full)
arg6 : attribute (mainly to put large circular buffer in ccm ram)


TODO: (30/10/2013) In buffer reservation mode, if a thread reserve a chunk : (varLenMsgQueueReserveChunk)
      and does not push it (varLenMsgQueueSendChunk), the circular buffer will be filled,
      and the entire queue will be stuck. A mecanism should be provided in this case to
      invalidate the chunk so that it will be discarded when sent (varLenMsgQueueSendChunk), and
      unlock the circular buffer so that the entire queue will not be stuck. To achive that,
      a small (number of simultaneous thread which send message with reserve api) table of
      current reserved chunk sould be included in struct VarLenMsgQueue.

      (31/10/2013) optimisation: varLenMsgQueuePushForZeroCopy has to be rewritten to avoid calls to
      reserveChunk/sendChunk API.


BUGS : static init of ccm ram array not done by gcc init stub

*/


#include "ch.h"
#include "hal.h"
#include "ringBuffer.h"

#ifdef __cplusplus
extern "C" {
#endif


#define ERROR_MAILBOX_FULL -10
#define ERROR_MAILBOX_FAIL -11
#define ERROR_MAILBOX_TIMEOUT -12
#define ERROR_CHUNKBUFFER_INVALID -20
#define ERROR_ZEROCOPY_NOT_ENABLED -30

typedef enum  {VarLenMsgQueue_REGULAR, VarLenMsgQueue_OUT_OF_BAND} VarLenMsgQueueUrgency;

#define _MAILBOX_BUFFER(name, size, attrib)				\
  static msg_t __ ## name ## _mailboxBufferInternal[size] attrib = {0};

//= {[0 ... size-1] = 0}
#define _SPARSECHUNK_MAP_BUFFER(name, size, attrib)				\
  static MsgPtrLen  __ ## name ## _sparseChunkMapBuffer[size] attrib = {[0 ... size-1] = {.msgPtrLen = 0}};

#define _VARLEN_MSGQUEUE_BUFFER(name, ringSize, mailboxSize, attrib)	\
_RINGBUFFER_BUFFER (name, ringSize, attrib);				\
_MAILBOX_BUFFER (name, mailboxSize, attrib);				\
_SPARSECHUNK_MAP_BUFFER(name, mailboxSize, attrib);

#define _VARLEN_MSGQUEUE_DATA(name, zeroCopy, ringSize, mailboxSize) {	\
  .mbAndSparseChunkSize = mailboxSize,				\
  .useZeroCopyApi = zeroCopy,					\
  .mbReservedSlot = 0,						\
  .mtx = _MUTEX_DATA(name.mtx),					\
  .mb = _MAILBOX_DATA(name.mb, __ ## name ## _mailboxBufferInternal, mailboxSize),	\
  .circBuf =  _RINGBUFFER_DATA(name, ringSize),			\
  .sparseChunkNumber = 0,							\
  .sparseChunkMap = __ ## name ## _sparseChunkMapBuffer				\
}

#define VARLEN_MSGQUEUE_DECL(scope, zeroCopy, name, ringSize, mailboxSize, attrib) \
  _VARLEN_MSGQUEUE_BUFFER (name, ringSize, mailboxSize, attrib)		\
  scope VarLenMsgQueue name = _VARLEN_MSGQUEUE_DATA(name, zeroCopy, ringSize, mailboxSize)


// used by zero copy PUSH (reserve/send) api
typedef struct  {
  uint8_t* bptr;  //  pointer on message buffer
  uint16_t blen;  //  length of message to send
} ChunkBuffer;

// used by zero copy POP (pop/free) api
typedef struct  {
  const uint8_t* const bptr; //  pointer on message buffer
  const uint16_t blen;       //  length of received message
} ChunkBufferRO;

typedef struct  VarLenMsgQueue VarLenMsgQueue;


/*
  goal : dynamicly initialise memory. shoud be used to workaround a bug if queue is located to ccmram
         not mandatory is queue is not in ccmram
  parameters : queue object
  return value: no
 */
void		varLenMsgDynamicInit   (VarLenMsgQueue* que);

/*
  goal : test if queue is full

  parameters : queue object
  return value: TRUE if queue if full, FALSE otherwise
 */
bool_t		varLenMsgQueueIsFull   (VarLenMsgQueue* que);

/*
  goal : test if queue is empty

  parameters : queue object
  return value:  TRUE if queue if empty, FALSE otherwise
 */
bool_t 		varLenMsgQueueIsEmpty  (VarLenMsgQueue* que);

/*
  goal :  return used size in circular buffer,
         does take into account used mailbox slots

  parameters : queue object
  return value: size in byte of used memory
 */
int32_t 	varLenMsgQueueUsedSize (VarLenMsgQueue* que);

/*
  goal : return free size in circular buffer,
         does take into account used mailbox slots

  parameters : queue object
  return value: size in byte of free memory
 */
int32_t 	varLenMsgQueueFreeSize (VarLenMsgQueue* que);

/*
  goal : push a new nessage of msgLen bytes. non blocking,
         if push is not possible, return error status

  parameters : queue object, pointer on buffer, message length,
               urgency  regular=queued at end of fifo or out_of_band queued at start of fifo
  return value: if > 0 : number of bytes actually sent
                if < 0 : error status (see errors at begining of this header file)
 */
int32_t 	varLenMsgQueuePush     (VarLenMsgQueue* que, const void* msg,
					const size_t msgLen,
					const VarLenMsgQueueUrgency urgency);

/*
  goal : pop a message. blocking

  parameters : queue object, pointer on buffer, buffer length
  return value:  if > 0 : number of actually copied bytes on buffer
                 if < 0 : error status (see errors at begining of this header file)
 */
int32_t 	varLenMsgQueuePop      (VarLenMsgQueue* que, void* msg,
					const size_t msgLen);

/*
  goal : pop a message. blocking, but with timeout parameter

  parameters : queue object, pointer on buffer, buffer length
	       timeout value
  return value:  if > 0 : number of actually copied bytes on buffer
                 if < 0 : error status (see errors at begining of this header file)
 */
int32_t 	varLenMsgQueuePopTimeout (VarLenMsgQueue* que, void* msg,
					  const size_t msgLen, const systime_t  time);




/*
  bugs : see BUGS section at begining of this header
  goal : (zero copy api)
         reserve a chunk of memory on the circular buffer to be filled eventually

  parameters : queue object, pointer to ChunkBuffer object, reserved buffer length
  return value: if > 0 : requested length
                if < 0 : error status (see errors at begining of this header file)
 */
int32_t		varLenMsgQueueReserveChunk (VarLenMsgQueue* que,
					    ChunkBuffer *cbuf, const size_t msgLen);

/*
  bugs : see BUGS section at begining of this header
  goal : (zero copy api)
         send a chunk of memory prevoiously reserved and filled as a message

  parameters : queue object, pointer to ChunkBuffer object,
               urgency  regular=queued at end of fifo or out_of_band queued at start of fifo
  return value: if > 0 : requested length
                if < 0 : error status (see errors at begining of this header file)
 */
int32_t		varLenMsgQueueSendChunk (VarLenMsgQueue* que, const ChunkBuffer *cbuf,
					 const VarLenMsgQueueUrgency urgency);


/*
  goal : (zero copy api)
  get a message to be processed without copy (blocking)

  parameters : INOUT queue object
	       OUT ChunkBufferRO
  return value: if > 0 : length of received msg
                if < 0 : error status (see errors at begining of this header file)
 */
uint32_t	varLenMsgQueuePopChunk (VarLenMsgQueue* que, ChunkBufferRO *cbro);

/*
  goal : (zero copy api)
  get a message to be processed without copy (blocking)

  parameters : INOUT queue object
	       OUT ChunkBufferRO
	       IN  timeout (or TIME_IMMEDIATE or TIME_INFINITE)
  return value: if > 0 : length od received msg
                if < 0 : error status (see errors at begining of this header file)

 */
uint32_t	varLenMsgQueuePopChunkTimeout (VarLenMsgQueue* que, ChunkBufferRO *cbro,
					       const systime_t  time);


/*
  goal : (zero copy api)
  free memory of a precedently given message by PopChunk

  parameters : INOUT queue object
               IN    ChunkBufferRO pointer
 */
void		varLenMsgQueueFreeChunk (VarLenMsgQueue* que, const ChunkBufferRO *cbuf);



/*
  goal : helper function to fix queue : verify that a empty queue is in coherent state

  parameters : queue object
  return value: TRUE if OK, FALSE if ERROR
 */
bool_t		varLenMsgQueueTestIntegrityIfEmpty(VarLenMsgQueue* que);

/*
  goal : give literal message from a status error code

  parameters : status error code
  return value: pointer to const message
 */
const char*     varLenMsgQueueError (int32_t messIndex);



/*
#                 _ __           _                    _
#                | '_ \         (_)                  | |
#                | |_) |  _ __   _   __   __   __ _  | |_     ___
#                | .__/  | '__| | |  \ \ / /  / _` | | __|   / _ \
#                | |     | |    | |   \ V /  | (_| | \ |_   |  __/
#                |_|     |_|    |_|    \_/    \__,_|  \__|   \___|
*/

typedef union {
  struct {
    uint16_t ptr;
    uint16_t len;
  };
  msg_t msgPtrLen;
} MsgPtrLen;


struct VarLenMsgQueue {
  const uint16_t mbAndSparseChunkSize;
  const bool_t useZeroCopyApi;
  uint16_t mbReservedSlot;
  Mutex mtx ;
  Mailbox mb;
  CircularBuffer circBuf;
  uint32_t sparseChunkNumber;
  MsgPtrLen * const sparseChunkMap;
} ;

#ifdef __cplusplus
}
#endif
