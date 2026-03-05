/*
 * General purpose circular buffer
 *
 * Copyright (C) 2021 Fabien-B <fabien-b@github.com> 
 *
 * This file is part of paparazzi. See LICENCE file.
 */

#include "utils/ring_buffer.h"
#include <string.h>


void ring_buffer_init(ring_buffer_t *rb, uint8_t *buf, size_t size) {
  rb->read_offset = 0;
  rb->write_offset = 0;
  rb->size = size;
  rb->buf = buf;
}

size_t ring_buffer_available(ring_buffer_t *rb) {
  if (rb->write_offset >= rb->read_offset) {
    return rb->write_offset - rb->read_offset;
  }
  else {
    return rb->size - (rb->read_offset - rb->write_offset);
  }
}

size_t ring_buffer_free_space(ring_buffer_t *ring_buffer) {
    return ring_buffer->size - ring_buffer_available(ring_buffer);
}

size_t ring_buffer_write(ring_buffer_t *rb, const uint8_t *data, size_t len) {
  size_t free_space = ring_buffer_free_space(rb);
  if (len > free_space) {
    len = free_space;
  }

  size_t first_chunk = rb->size - rb->write_offset;
  if (first_chunk > len) {
    first_chunk = len;
  }

  // Copy first chunk
  memcpy(&rb->buf[rb->write_offset], data, first_chunk);

  // Copy second chunk if wrap needed
  size_t remaining = len - first_chunk;
  if (remaining > 0) {
    memcpy(&rb->buf[0], data + first_chunk, remaining);
  }

  rb->write_offset = (rb->write_offset + len) % rb->size;

  return len;
}

size_t ring_buffer_read(ring_buffer_t *rb, uint8_t *read_buffer, size_t len)
{
  size_t available = ring_buffer_available(rb);
  if (len > available) {
    len = available;
  }

  // how much until wrap?
  size_t first_chunk = rb->size - rb->read_offset;
  if (first_chunk > len) {
    first_chunk = len;
  }

  memcpy(read_buffer, &rb->buf[rb->read_offset], first_chunk);

  size_t remaining = len - first_chunk;
  if (remaining > 0) {
    memcpy(read_buffer + first_chunk, &rb->buf[0], remaining);
  }

  rb->read_offset = (rb->read_offset + len) % rb->size;

  return len;
}
