#ifndef __RING_BUFFER__
#define __RING_BUFFER__


/*

TODO:


*/

#ifdef DEBUG_ON_WORKSTATION
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#else
#include "std.h"
#endif
#include <unistd.h>

#define ERROR_CIRCULAR_BUFFER_FULL -1
#define ERROR_CIRCULAR_BUFFER_UNSYNC -2

#define _RINGBUFFER_DATA(name, _size) {	        \
  .writePointer = 0,				\
  .readPointer = 0,				\
  .size = _size,				\
  .keys = __ ## name ## _ringBufferInternal	\
}

#define _RINGBUFFER_BUFFER(name, size, attrib)	\
  static uint8_t __ ## name ## _ringBufferInternal[size] attrib = {0};

#define RINGBUFFER_DECL(scope, name, size, attrib)			\
  _RINGBUFFER_BUFFER(name, size, attrib);			\
  scope CircularBuffer name = _RINGBUFFER_DATA(name, size)


typedef struct
{
  volatile uint16_t writePointer; /* write pointer */
  volatile uint16_t readPointer;  /* read pointer */
  const    uint16_t size;         /* size of circular buffer */
  uint8_t * const   keys;    /* pointer to Element of circular buffer */
} CircularBuffer;




bool ringBufferIsFull(const CircularBuffer* que);
bool ringBufferIsEmpty(const CircularBuffer* que);
int32_t ringBufferUsedSize(const CircularBuffer* que);
int32_t ringBufferFreeSize(const CircularBuffer* que);
int32_t ringBufferEnqueBuffer(CircularBuffer* que, const uint8_t* pK, size_t len);
int32_t ringBufferDequeBuffer(CircularBuffer* que, uint8_t* pK, size_t len);
int32_t ringBufferCopyFromAddr(CircularBuffer* que, const uint16_t readFrom,
    uint8_t* pK, size_t len);

static inline uint16_t ringBufferGetSize(const CircularBuffer* que) {
  return que->size;
}

static inline uint16_t ringBufferGetWritePointer(const CircularBuffer* que) {
  return que->writePointer;
}

static inline void ringBufferSetWritePointer(CircularBuffer* que, uint16_t writePointer) {
  que->writePointer = writePointer;
}

static inline uint16_t ringBufferGetReadPointer(const CircularBuffer* que) {
  return que->readPointer;
}

static inline void ringBufferSetReadPointer(CircularBuffer* que, uint16_t readPointer) {
  que->readPointer = readPointer;
}

static inline void ringBufferReadSeek(CircularBuffer* que, size_t len) {
  que->readPointer = (que->readPointer+len) % que->size;
}

static inline void ringBufferWriteSeek(CircularBuffer* que, size_t len) {
  que->writePointer = (que->writePointer+len) % que->size;
}

static inline int32_t ringBufferFreeSizeToEndOfCircular(const CircularBuffer* que)
{
  return (que->size - que->writePointer);
}

static inline uint8_t *ringBufferGetAddrOfElem(const CircularBuffer* que, const uint16_t elem)
{
  return (&que->keys[elem]);
}

static inline uint16_t ringBufferGetIndexOfElemAddr(const CircularBuffer* que, const uint8_t* elemAddr)
{
  return (elemAddr - que->keys);
}



#endif
