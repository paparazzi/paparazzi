#include "ringBuffer.h"

#include <string.h>


/*

TODO:

 * CircularBufferInit statique avec une macro pour eviter le malloc_m


 * encapsuler dans une structure avec une mailbox chibios
 * voir pb atomicité arithmetique sur les pointeurs entre
 l'increment et le modulo
 * peut-être réglé avec le & logique si on se restreint à des tailles modulo 2
 * optimisation avec memcopy au lieu de copie octets par octets

*/

// RING_BUFFER_DECL (, toto, 10,);
// RING_BUFFER_DECL (static, toto, 10, __attribute__ ((section(".ccmram"), aligned(8))));

/**< Init Circular Buffer */

bool ringBufferIsFull(const CircularBuffer* que)
{
  return ringBufferFreeSize(que) == 0;
}

bool ringBufferIsEmpty(const CircularBuffer* que)
{
  return (que->readPointer == que->writePointer);
}

int32_t ringBufferFreeSize(const CircularBuffer* que)
{
  if (que->writePointer < que->readPointer) {
    return que->readPointer - que->writePointer -1;
  } else {
    return (que->size -1) - (que->writePointer - que->readPointer);
  }
}


int32_t ringBufferUsedSize(const CircularBuffer* que)
{
  return (que->size -1) - ringBufferFreeSize (que);
}


int32_t ringBufferEnqueBuffer(CircularBuffer* que, const uint8_t* pK, size_t len)
{
  if (len > (size_t) ringBufferFreeSize (que)) {
    return ERROR_CIRCULAR_BUFFER_FULL;
  }

  if (que->writePointer < que->readPointer) {
    // write pointer is before read pointer
    // only one memcpy (area don't overlap)
    memcpy (&que->keys[que->writePointer], pK, len);
    ringBufferWriteSeek (que, len);
  } else {
    // write pointer is equal or after read point
    // depending on size to write, we will do one or two memcpy
    if (len <= (size_t) (que->size - que->writePointer)) {
      // size to write is less or equal to size from write pointer to end of buffer
      // just one memcpy
      memcpy (&que->keys[que->writePointer], pK, len);
      ringBufferWriteSeek (que, len);
    } else {
      // size to write is less or equal more than size from write pointer to end of buffer
      // two memcpy
      const uint32_t firstCopyLen = que->size - que->writePointer;
      const uint32_t secondCopyLen = len - firstCopyLen;
      memcpy (&que->keys[que->writePointer], pK, firstCopyLen);
      memcpy (&que->keys[0], &pK[firstCopyLen], secondCopyLen);
      que->writePointer = secondCopyLen;
    }
  }

  return len;
}


int32_t ringBufferDequeBuffer(CircularBuffer* que, uint8_t* pK, size_t len)
{
  if (len > (size_t) ringBufferUsedSize (que)) {
    return ERROR_CIRCULAR_BUFFER_FULL;
  }

  if (que->readPointer < que->writePointer) {
    // read pointer is before write pointer
    // only one memcpy (area don't overlap)
    memcpy (pK, &que->keys[que->readPointer], len);
    ringBufferReadSeek (que, len);
  } else {
    // read pointer is equal or after write pointer
    // depending on size to read, we will do one or two memcpy
    if (len <= (size_t) (que->size - que->readPointer)) {
      // size to read is less or equal to size from read pointer to end of buffer
      // just one memcpy
      memcpy (pK, &que->keys[que->readPointer], len);
      ringBufferReadSeek (que, len);
    } else {
      // size to read is less or equal more than size from read pointer to end of buffer
      // two memcpy
      const uint32_t firstCopyLen = que->size - que->readPointer;
      const uint32_t secondCopyLen = len - firstCopyLen;
      memcpy (pK, &que->keys[que->readPointer], firstCopyLen);
      memcpy (&pK[firstCopyLen], &que->keys[0], secondCopyLen);
      que->readPointer = secondCopyLen;
    }
  }

  return len;
}

int32_t ringBufferCopyFromAddr(CircularBuffer* que, const uint16_t readFrom,
    uint8_t* pK, size_t len)
{
  if (readFrom < que->writePointer) {
    // read pointer is before write pointer
    // only one memcpy (area don't overlap)
    memcpy (pK, &que->keys[readFrom], len);
  } else {
    // read pointer is equal or after write pointer
    // depending on size to read, we will do one or two memcpy
    if (len <= (size_t) (que->size - readFrom)) {
      // size to read is less or equal to size from read pointer to end of buffer
      // just one memcpy
      memcpy (pK, &que->keys[readFrom], len);
    } else {
      // size to read is less or equal more than size from read pointer to end of buffer
      // two memcpy
      const uint32_t firstCopyLen = que->size - readFrom;
      const uint32_t secondCopyLen = len - firstCopyLen;
      memcpy (pK, &que->keys[readFrom], firstCopyLen);
      memcpy (&pK[firstCopyLen], &que->keys[0], secondCopyLen);
    }
  }

  return len;
}
