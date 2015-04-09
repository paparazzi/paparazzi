#include "ch.h"
#include "hal.h"
#include "varLengthMsgQ.h"
#include <string.h>

#ifdef DEBUG_VARLEN_MSGQ
#include "stdutil.h"
#include "globalVar.h"
#else
#define DebugTrace(...)
#endif //  DEBUG_VARLEN_MSGQ



static void       varLenMsgQueueLock(VarLenMsgQueue* que);
static void       varLenMsgQueueUnlock(void);
static uint16_t   popSparseChunkMap (VarLenMsgQueue* que, const uint16_t  mplAddr);
static void       pushSparseChunkMap (VarLenMsgQueue* que, const MsgPtrLen mpl);
static uint16_t   getSparseChunkMapFirstFreeIndex (const VarLenMsgQueue* que);
static int32_t 	  varLenMsgQueuePushForZeroCopy (VarLenMsgQueue* que, const void* msg,
						 const size_t msgLen,
						 const VarLenMsgQueueUrgency urgency);


// Work around of still to be resolved bug : static init of array don't work in
// ccmram, so this init function has to be called at init.
void		varLenMsgDynamicInit   (VarLenMsgQueue* que)
{
  memset (que->sparseChunkMap, 0, que->mbAndSparseChunkSize*sizeof(MsgPtrLen));
  que->sparseChunkNumber=0;
}

bool_t		varLenMsgQueueIsFull (VarLenMsgQueue* que)
{
  varLenMsgQueueLock (que);
  bool_t retVal = ringBufferIsFull (&que->circBuf) || chMBGetFreeCountI (&que->mb) <= 0;
  varLenMsgQueueUnlock ();
  return retVal;
}

bool_t 		varLenMsgQueueIsEmpty (VarLenMsgQueue* que)
{
  varLenMsgQueueLock (que);
  bool_t retVal = ringBufferIsEmpty (&que->circBuf) && chMBGetUsedCountI (&que->mb) <= 0;
  varLenMsgQueueUnlock ();
  return retVal;
}

int32_t varLenMsgQueueUsedSize (VarLenMsgQueue* que)
{
  varLenMsgQueueLock (que);
  int32_t  retVal =  ringBufferUsedSize (&que->circBuf);
  varLenMsgQueueUnlock ();
  return retVal;
}

int32_t varLenMsgQueueFreeSize (VarLenMsgQueue* que)
{
  varLenMsgQueueLock (que);
  int32_t  retVal =  ringBufferFreeSize (&que->circBuf);
  varLenMsgQueueUnlock ();
  return retVal;
}


static int32_t varLenMsgQueuePushForZeroCopy(VarLenMsgQueue* que, const void* msg, const size_t msgLen,
    const VarLenMsgQueueUrgency urgency)
{
  int32_t retVal;
  ChunkBuffer cbuf;

  retVal = varLenMsgQueueReserveChunk (que, &cbuf, msgLen);
  if (retVal < 0)
    return retVal;

  memcpy (cbuf.bptr, msg, msgLen);
  retVal = varLenMsgQueueSendChunk (que, &cbuf, urgency);

  return retVal;
}


int32_t varLenMsgQueuePush(VarLenMsgQueue* que, const void* msg, const size_t msgLen,
    const VarLenMsgQueueUrgency urgency)
{
  // if we use zeroCopyApi, whe sould only expose linear address buffer, so we do not
  // allow message splitted between end end of buffer ans start of buffer
  if (que->useZeroCopyApi == TRUE) {
    return varLenMsgQueuePushForZeroCopy (que, msg, msgLen, urgency);
  } else {
    msg_t   postRet;
    int32_t retVal;

    varLenMsgQueueLock (que);
    const uint16_t writeIdx = ringBufferGetWritePointer(&que->circBuf);

    if (chMBGetFreeCountI (&que->mb) <= que->mbReservedSlot) {
      retVal = ERROR_MAILBOX_FULL;
      goto  unlockAndExit;
    }

    retVal = ringBufferEnqueBuffer(&que->circBuf, msg, msgLen);

    if (retVal != ERROR_CIRCULAR_BUFFER_FULL) {
      const MsgPtrLen mpl = {{
        .ptr = writeIdx,
        .len = (uint16_t) msgLen
      }};

      // if mailbox is too small, we should remove the writen message from circular buffer
      // and return error message
      if (urgency == VarLenMsgQueue_REGULAR) {
        postRet = chMBPost (&que->mb, mpl.msgPtrLen, TIME_IMMEDIATE);
      } else {
        postRet = chMBPostAhead (&que->mb, mpl.msgPtrLen, TIME_IMMEDIATE);
      }
      if (postRet != RDY_OK) {
        retVal = ERROR_MAILBOX_FAIL;
        ringBufferSetWritePointer (&que->circBuf, writeIdx);
      }
    }
unlockAndExit:
    varLenMsgQueueUnlock ();
    return retVal;
  }
}

int32_t varLenMsgQueuePop(VarLenMsgQueue* que, void* msg, const size_t msgLen)
{
  return varLenMsgQueuePopTimeout (que, msg, msgLen, TIME_INFINITE);
}


int32_t varLenMsgQueuePopTimeout (VarLenMsgQueue* que, void* msg,
					  const size_t msgLen, const systime_t  time)
{
  int32_t retVal, status;
  MsgPtrLen mpl;

  varLenMsgQueueLock (que);

  if ((status = chMBFetch (&que->mb, &mpl.msgPtrLen, time) != RDY_OK)) {
    if (status == RDY_TIMEOUT)
      retVal = ERROR_MAILBOX_TIMEOUT;
    else
      retVal = ERROR_MAILBOX_FAIL;
    goto unlockAndExit;
  }


  if (mpl.ptr != ringBufferGetReadPointer(&que->circBuf)) {
    // OUT OF BAND CONDITION
    pushSparseChunkMap (que, mpl);
    //    DebugTrace ("pushSparseChunkMap (ptr=%d len=%d)", mpl.ptr, mpl.len);
    //    oobCondition=TRUE;
    const uint16_t sizeToCopy = msgLen < mpl.len ? msgLen : mpl.len;
    retVal =  ringBufferCopyFromAddr(&que->circBuf, mpl.ptr, msg, sizeToCopy);
  } else {
    // if buffer is smaller than message (too bad)
    if (msgLen < mpl.len) {
      // Don't overwrite destination buffer
      retVal =  ringBufferDequeBuffer(&que->circBuf, msg, msgLen);
      // but simulate reading of entire message so that readPointer is still valid at a msg boundary
      ringBufferReadSeek (&que->circBuf, mpl.len-msgLen);
    } else {
      // destination buffer is well sized
      retVal =  ringBufferDequeBuffer(&que->circBuf, msg, mpl.len);
    }
  }

  while (que->sparseChunkNumber > 0) {
    const uint16_t actualReadPpointer=ringBufferGetReadPointer(&que->circBuf);
    const uint16_t lenOfChunk = popSparseChunkMap (que, actualReadPpointer);
    //    DebugTrace ("lenOfChunk(R=%d) = %d", actualReadPpointer, lenOfChunk);
    // if address is not found, do not search anymore and exit loop
    if (lenOfChunk == 0)
      break;
    // address is found, we are on an already read oob msg, let seek

    ringBufferReadSeek(&que->circBuf, lenOfChunk);
  }

unlockAndExit:
  varLenMsgQueueUnlock ();
  return retVal;
}

// WARNING NOT FULLY REENTRANT ALGORITHM
// do not take into account when a reserved chunk is not sended, so queue will be stuck full if
// a thread take to much time between varLenMsgQueueReserveChunk and varLenMsgQueueSendChunk
// a mecanism should be provided which will discard chunk if circular buffer rollback on this chunk
int32_t varLenMsgQueueReserveChunk (VarLenMsgQueue* que, ChunkBuffer *cbuf, const size_t msgLen)
{
  int32_t retVal = msgLen;
  varLenMsgQueueLock(que);

  // if there is not enough room, return
  const uint16_t freeSize=ringBufferFreeSize (&que->circBuf);
  if (msgLen > freeSize) {
    retVal=ERROR_CIRCULAR_BUFFER_FULL;
    cbuf->bptr=NULL;
    goto unlockAndExit;
  }

  if (chMBGetFreeCountI (&que->mb) <= que->mbReservedSlot) {
    retVal = ERROR_MAILBOX_FULL;
    cbuf->bptr=NULL;
    goto  unlockAndExit;
  }


  // we want go give back a linear buffer, so if there is no enough room between write pointer and
  // end of buffer, sacrify this room, mark it as sparsechunk, and give room from begining of buffer
  // if there is enough room for this

  const uint16_t sizeToEnd= ringBufferFreeSizeToEndOfCircular (&que->circBuf);
  //  DebugTrace ("sizeToEnd = %d", sizeToEnd);
  if (msgLen > sizeToEnd) {
    //    DebugTrace ("msgLen(%d) > sizeToEnd(%d)", msgLen, sizeToEnd);
    if ((msgLen + sizeToEnd) > freeSize) {
      //      DebugTrace ("ERROR_CIRCULAR_BUFFER_FULL");
      retVal=ERROR_CIRCULAR_BUFFER_FULL;
      cbuf->bptr=NULL;
      goto unlockAndExit;
    } else {
      const MsgPtrLen mpl = {{
        .ptr = ringBufferGetWritePointer(&que->circBuf),
        .len = sizeToEnd
      }};
      pushSparseChunkMap (que, mpl);
      ringBufferSetWritePointer(&que->circBuf, 0);
    }
  }

  // increment count of pre-reserved buffers
  que->mbReservedSlot++;
  // give pre-reserved buffer
  cbuf->bptr= ringBufferGetAddrOfElem(&que->circBuf, ringBufferGetWritePointer(&que->circBuf));
  cbuf->blen= (const uint16_t) msgLen;
  /* DebugTrace ("pre write seek R=%d, W=%d",  */
  /*       ringBufferGetReadPointer(&que->circBuf), */
  /*       ringBufferGetWritePointer(&que->circBuf)); */
  ringBufferWriteSeek (&que->circBuf, msgLen);
  /* DebugTrace ("post write seek R=%d, W=%d",  */
  /*       ringBufferGetReadPointer(&que->circBuf), */
  /*       ringBufferGetWritePointer(&que->circBuf)); */

unlockAndExit:
  varLenMsgQueueUnlock();
  return retVal;
}


int32_t varLenMsgQueueSendChunk (VarLenMsgQueue* que, const ChunkBuffer *cbuf,
    const VarLenMsgQueueUrgency urgency)
{
  int32_t retVal = cbuf->blen;
  msg_t   postRet;


  if (cbuf->bptr == NULL)
    return ERROR_CHUNKBUFFER_INVALID;

  varLenMsgQueueLock(que);

  const MsgPtrLen mpl = {{
      .ptr = ringBufferGetIndexOfElemAddr(&que->circBuf, cbuf->bptr),
      .len = cbuf->blen
  }};

  if (urgency == VarLenMsgQueue_REGULAR) {
    postRet = chMBPost (&que->mb, mpl.msgPtrLen, TIME_IMMEDIATE);
  } else {
    postRet = chMBPostAhead (&que->mb, mpl.msgPtrLen, TIME_IMMEDIATE);
  }

  if (postRet != RDY_OK) {
    retVal = ERROR_MAILBOX_FAIL;
    // mark previously buffered message in reserved area as sparse chunk so that it will be
    // ignored
    pushSparseChunkMap (que, mpl);
  } else {
    que->mbReservedSlot--;
  }

  varLenMsgQueueUnlock();
  return retVal;
}

uint32_t varLenMsgQueuePopChunk (VarLenMsgQueue* que, ChunkBufferRO *cbro)
{
  return varLenMsgQueuePopChunkTimeout (que,  cbro, TIME_INFINITE);
}


uint32_t varLenMsgQueuePopChunkTimeout (VarLenMsgQueue* que, ChunkBufferRO *cbro,
					       const systime_t  time)
{
  MsgPtrLen mpl ;
  ChunkBuffer *cbrw = (ChunkBuffer *) cbro;


  // cannot use this function if queue is not declared zero copy compliant
  if (que->useZeroCopyApi == FALSE) {
    cbrw->blen=0;
    cbrw->bptr = NULL;
    return ERROR_ZEROCOPY_NOT_ENABLED;
  }
  if (chMBFetch (&que->mb, &mpl.msgPtrLen, time) == RDY_OK) {
    cbrw->blen=mpl.len;
    cbrw->bptr = ringBufferGetAddrOfElem(&que->circBuf, mpl.ptr);
    return mpl.len;
  } else {
    cbrw->blen=0;
    cbrw->bptr = NULL;
    return ERROR_MAILBOX_TIMEOUT;
  }
}

void varLenMsgQueueFreeChunk (VarLenMsgQueue* que, const ChunkBufferRO *cbuf)
{
  varLenMsgQueueLock (que);
  const MsgPtrLen mpl = {.len = cbuf->blen, .ptr = ringBufferGetIndexOfElemAddr(&que->circBuf, cbuf->bptr)};

  if (mpl.ptr != ringBufferGetReadPointer(&que->circBuf)) {
    // OUT OF BAND CONDITION
    pushSparseChunkMap (que, mpl);
  } else {
    // simulate reading of entire message so that readPointer is still valid at a msg boundary
    ringBufferReadSeek (&que->circBuf, mpl.len);
  }

  while (que->sparseChunkNumber > 0) {
    const uint16_t actualReadPpointer=ringBufferGetReadPointer(&que->circBuf);
    const uint16_t lenOfChunk = popSparseChunkMap (que, actualReadPpointer);
    //    DebugTrace ("lenOfChunk(R=%d) = %d", actualReadPpointer, lenOfChunk);
    // if address is not found, do not search anymore and exit loop
    if (lenOfChunk == 0)
      break;
    // address is found, we are on an already read oob msg, let seek

    ringBufferReadSeek(&que->circBuf, lenOfChunk);
  }

  varLenMsgQueueUnlock ();
}



static void varLenMsgQueueLock(VarLenMsgQueue* que)
{
  chMtxLock(&que->mtx);
}


static void varLenMsgQueueUnlock()
{
  chMtxUnlock();
}


static uint16_t getSparseChunkMapFirstFreeIndex(const VarLenMsgQueue* que)
{
  for (uint16_t i=0; i<que->mbAndSparseChunkSize; i++) {
    if (que->sparseChunkMap[i].msgPtrLen == 0) {
      return i;
    }
  }
  return 0;
}

static void pushSparseChunkMap (VarLenMsgQueue* que, const MsgPtrLen mpl)
{
  const uint16_t nextNextFreeIndex = getSparseChunkMapFirstFreeIndex(que);
  que->sparseChunkMap[nextNextFreeIndex] = mpl;
  que->sparseChunkNumber++;
}


static uint16_t popSparseChunkMap (VarLenMsgQueue* que, const uint16_t  mplAddr)
{
  uint16_t associatedLen=0;
  for (uint16_t i=0; i<que->mbAndSparseChunkSize; i++) {
    if ((que->sparseChunkMap[i].ptr == mplAddr) && (que->sparseChunkMap[i].msgPtrLen)) {
      associatedLen = que->sparseChunkMap[i].len;
      que->sparseChunkMap[i].msgPtrLen = 0;
      que->sparseChunkNumber--;
      break;
    }
  }
  return associatedLen;
}

bool_t varLenMsgQueueTestIntegrityIfEmpty(VarLenMsgQueue* que)
{
  bool_t retVal = TRUE;
  varLenMsgQueueLock(que);
  int32_t status;

  if ((status = chMBGetUsedCountI (&que->mb)) > 0) {
    DebugTrace ("Error: mailbox not empty : [%d]", status);
    retVal=FALSE;
    goto unlockAndExit;
  }

  if (! ringBufferIsEmpty (&que->circBuf)) {
    DebugTrace ("Error: circular buffer not empty");
    retVal=FALSE;
    goto unlockAndExit;
  }

  if (que->sparseChunkNumber != 0) {
    DebugTrace ("Error: sparseChunkNumber not NULL");
    retVal=FALSE;
    goto unlockAndExit;
  }

  if (que->mbReservedSlot != 0) {
    DebugTrace ("Error: mbReservedSlot not NULL");
    retVal=FALSE;
    goto unlockAndExit;
  }

  // parse  sparseChunkMap
  for (uint16_t i=0; i< que->mbAndSparseChunkSize; i++)  {
    if (que->sparseChunkMap[i].len != 0) {
      DebugTrace ("Error: sparseChunkMap not erased");
      retVal=FALSE;
      goto unlockAndExit;
    }
  }

unlockAndExit:
  varLenMsgQueueUnlock();
  return retVal;
}

const char*     varLenMsgQueueError (int32_t messIndex)
{
  switch (messIndex) {
    case ERROR_MAILBOX_FULL: return "mailbox full";
    case ERROR_MAILBOX_FAIL: return "mailbox fail";
    case ERROR_CHUNKBUFFER_INVALID: return "chunkbuffer invalid";
    case ERROR_CIRCULAR_BUFFER_FULL: return "circular buffer full";
    case ERROR_CIRCULAR_BUFFER_UNSYNC: return "circular buffer unsync";
  }

  return "unknown error";
}


/*

   =========SCENARIO 1

   initial

   0 1 2 3 4 5 6 7 8 9
   |r| | | | | | | | | |
   ---------------------
   |w| | | | | | | | | |

   after write 3

   0 1 2 3 4 5 6 7 8 9
   |.|.|.|w| | | | | | |
   ---------------------
   |r| | | | | | | | | |

   after write out of band 2

   0 1 2 3 4 5 6 7 8 9
   |.|.|.|O|O|w| | | | |
   ---------------------
   |r| | | | | | | | | |

   after reading out of band 2:
   msg.ptr(3) != readPtr(0) so OUT OF BAND CONDITION
 * let sparseChunkNumber=1
 * let sparseChunkMap[0] = .len=2 .ptr=3
 * keep readPtr @ 0

 0 1 2 3 4 5 6 7 8 9
 |.|.|.|o|o|w| | | | |
 ---------------------
 |r| | | | | | | | | |


after reading reglar first message
msg.ptr(0) == readPtr(0) so REGULAR Message
update readPtr to 3
since sparseChunkNumber, verify if readPtr is found in sparseChunkMap :
YES : readPtr += len(2) [5]
sparseChunkNumber=0
sparseChunkMap[0] = .len=0 .ptr=0

 0 1 2 3 4 5 6 7 8 9
| | | | | |w | | | | |
---------------------
| | | | | |r| | | | |


=========SCENARIO 2

initial

 0 1 2 3 4 5 6 7 8 9
|r| | | | | | | | | |
---------------------
|w| | | | | | | | | |

after write 3

 0 1 2 3 4 5 6 7 8 9
|.|.|.|w| | | | | | |
---------------------
|r| | | | | | | | | |

after write out of band 2

 0 1 2 3 4 5 6 7 8 9
|.|.|.|O|O|w| | | | |
---------------------
|r| | | | | | | | | |

after another write out of band 2

 0 1 2 3 4 5 6 7 8 9
|.|.|.|O|O|O|O|w| | |
---------------------
|r| | | | | | | | | |

after reading LAST out of band 2:
  msg.ptr(5) != readPtr(0) so OUT OF BAND CONDITION
  * let sparseChunkNumber++ (1)
  * let sparseChunkMap[0] = .len=2 .ptr=5
  * keep readPtr @ 0

 0 1 2 3 4 5 6 7 8 9
|.|.|.|O|O|o|o|w| | |
---------------------
|r| | | | | | | | | |

after reading FIRST out of band 2:
  msg.ptr(3) != readPtr(0) so OUT OF BAND CONDITION
  * let sparseChunkNumber++ (2)
  * let sparseChunkMap[1] = .len=2 .ptr=3
  * keep readPtr @ 0

 0 1 2 3 4 5 6 7 8 9
|.|.|.|o|o|o|o|w| | |
---------------------
|r| | | | | | | | | |


after reading reglar first message msg.ptr(0) == readPtr(0) so REGULAR
Message update readPtr to 3 since sparseChunkNumber!=0, verify if
readPtr is found in sparseChunkMap : YES : readPtr += len(2) => [5]
sparseChunkNumber=1 since sparseChunkNumber!=0, verify if readPtr is
found in sparseChunkMap : YES : readPtr += len(2) => [7]
sparseChunkNumber=0 sparseChunkMap[0] = .len=0 .ptr=0

 0 1 2 3 4 5 6 7 8 9
| | | | | | | |w| | |
---------------------
| | | | | | | |r| | |

=========SCENARIO 3

initial

 0 1 2 3 4 5 6 7 8 9
|r| | | | | | | | | |
---------------------
|w| | | | | | | | | |

after write 3

 0 1 2 3 4 5 6 7 8 9
|.|.|.|w| | | | | | |
---------------------
|r| | | | | | | | | |

after write out of band 2

 0 1 2 3 4 5 6 7 8 9
|.|.|.|O|O|w| | | | |
---------------------
|r| | | | | | | | | |

after another write out of band 2

 0 1 2 3 4 5 6 7 8 9
|.|.|.|O|O|O|O|w| | |
---------------------
|r| | | | | | | | | |

after reading FIRST out of band 2:
  msg.ptr(3) != readPtr(0) so OUT OF BAND CONDITION
  * let sparseChunkNumber++ (1)
  * let sparseChunkMap[0] = .len=2 .ptr=3
  * keep readPtr @ 0

 0 1 2 3 4 5 6 7 8 9
|.|.|.|o|o|O|O|w| | |
---------------------
|r| | | | | | | | | |

after reading FIRST out of band 2:
  msg.ptr(5) != readPtr(0) so OUT OF BAND CONDITION
  * let sparseChunkNumber++ (2)
  * let sparseChunkMap[1] = .len=2 .ptr=5
  * keep readPtr @ 0

 0 1 2 3 4 5 6 7 8 9
|.|.|.|o|o|o|o|w| | |
---------------------
|r| | | | | | | | | |


after reading reglar first message
msg.ptr(0) == readPtr(0) so REGULAR Message
update readPtr to 3
since sparseChunkNumber!=0, verify if readPtr is found in sparseChunkMap :
YES : readPtr += len(2) => [5]
sparseChunkNumber=1
since sparseChunkNumber!=0, verify if readPtr is found in sparseChunkMap :
YES : readPtr += len(2) => [7]
sparseChunkNumber=0
sparseChunkMap[0] = .len=0 .ptr=0

 0 1 2 3 4 5 6 7 8 9
| | | | | | | |w| | |
---------------------
| | | | | | | |r| | |

================================================

=========SCENARIO 4

initial

 0 1 2 3 4 5 6 7 8 9
|r| | | | | | | | | |
---------------------
|w| | | | | | | | | |

after write 3

 0 1 2 3 4 5 6 7 8 9
|.|.|.|w| | | | | | |
---------------------
|r| | | | | | | | | |

after write out of band 2

 0 1 2 3 4 5 6 7 8 9
|.|.|.|O|O|w| | | | |
---------------------
|r| | | | | | | | | |

after write 2

 0 1 2 3 4 5 6 7 8 9
|.|.|.|O|O|.|.|w| | |
---------------------
|r| | | | | | | | | |

after reading (OOB get from queue):
  msg.ptr(3) != readPtr(0) so OUT OF BAND CONDITION
  * let sparseChunkNumber++ (1)
  * let sparseChunkMap[0] = .len=2 .ptr=5
  * keep readPtr @ 0

 0 1 2 3 4 5 6 7 8 9
|.|.|.|o|o|.|.|w| | |
---------------------
|r| | | | | | | | | |

after reading FIRST normal message:
  msg.ptr(0) == readPtr(0) so REGULAR MESSAGE
  update readPtr to 3
  since sparseChunkNumber!=0, verify if readPtr is found in sparseChunkMap :
  YES : readPtr += len(2) => [5]
  sparseChunkNumber=0

 0 1 2 3 4 5 6 7 8 9
| | | | | |.|.|w| | |
---------------------
| | | | | |r| | | | |

after reading SECOND normal message:
  msg.ptr(5) == readPtr(5) so REGULAR MESSAGE
  update readPtr to 7


=========SCENARIO 5

initial

 0 1 2 3 4 5 6 7 8 9
|r| | | | | | | | | |
---------------------
|w| | | | | | | | | |


after write regular 1

 0 1 2 3 4 5 6 7 8 9
|.|w| | | | | | | | |
---------------------
|r| | | | | | | | | |


after write out of band 1

 0 1 2 3 4 5 6 7 8 9
|.|O|w| | | | | | | |
---------------------
|r| | | | | | | | | |

after write  out of band 1

 0 1 2 3 4 5 6 7 8 9
|.|O|O|w| | | | | | |
---------------------
|r| | | | | | | | | |

after reading (last OOB get from queue):
  msg.ptr(2) != readPtr(0) so OUT OF BAND CONDITION
  * let sparseChunkNumber++ (1)
  * let sparseChunkMap[0] = .ptr=2 .len=1
  * keep readPtr @ 0

 0 1 2 3 4 5 6 7 8 9
|.|O|o|w| | | | | | |
---------------------
|r| | | | | | | | | |

after reading (first OOB get from queue):
  msg.ptr(1) != readPtr(0) so OUT OF BAND CONDITION
  * let sparseChunkNumber++ (2)
  * let sparseChunkMap[1] = .ptr=1 .len=1
  * keep readPtr @ 0

 0 1 2 3 4 5 6 7 8 9
|.|o|o|w| | | | | | |
---------------------
|r| | | | | | | | | |


after reading FIRST normal message:
  msg.ptr(0) == readPtr(0) so REGULAR MESSAGE
  update readPtr to 1
  since sparseChunkNumber!=0, verify if readPtr is found in sparseChunkMap :
  YES : readPtr += len(1) => [2]
  sparseChunkNumber=1
  since sparseChunkNumber!=0, verify if readPtr is found in sparseChunkMap :
  YES : readPtr += len(1) => [3]
  sparseChunkNumber=0


======================
 0 1 2 3 4 5 6 7 8 9
| | | | | | | | | | |
---------------------
| | | | | | | | | | |


 */
