/*
 * Copyright (C) 2021 Fabien-B <fabien-b@github.com> 
 *
 * This file is part of paparazzi. See LICENCE file.
 */

#include "stdio.h"
#include "stdlib.h"
#include "framed_ring_buffer.h"
#include <string.h>
#include "tap.h"

uint8_t buffer[20];
struct framed_ring_buffer cbuf;


int main(int argc __attribute_maybe_unused__, char **argv __attribute_maybe_unused__)
{
  uint8_t src[20];
  for(int i=0; i<20; i++) {
    src[i] = 'A' + i;
  }
  uint8_t dst[20];

  note("running circular buffer tests");
  plan(23);

  framed_ring_buffer_init(&cbuf, buffer, sizeof(buffer));

  // get on an empty buffer
  int ret = framed_ring_buffer_get(&cbuf, dst, sizeof(dst));
  ok(ret == CIR_ERROR_NO_MSG, "expected CIR_ERROR_NO_MSG, got %d", ret);


  // put 10 bytes. 12 bytes will be occupied, 8 bytes free, 5 available for the next buffer
  ret = framed_ring_buffer_put(&cbuf, src, 10);
  size_t available = framed_ring_buffer_available(&cbuf);
  ok(ret == 0, "expected 0, got %d\n", ret);
  ok(available == 5, "expected 5 bytes available, got %lu", available);


  // put 3 bytes. 12+5=17/20 bytes occupied, 3 left. 0 available for the next buffer
  ret = framed_ring_buffer_put(&cbuf, src, 3);
  available = framed_ring_buffer_available(&cbuf);
  ok(ret == 0, "expected 0, got %d\n", ret);
  ok(available == 0, "expected 0 bytes available, got %lu", available);
  // try puting 1 byte : should fail
  ret = framed_ring_buffer_put(&cbuf, src, 1);
  ok(ret == CIR_ERROR_NO_SPACE_AVAILABLE, "expected CIR_ERROR_NO_SPACE_AVAILABLE, got %d\n", ret);

  // drop last buffer
  framed_ring_buffer_drop_last(&cbuf);
  // put 4 bytes. 12+6=18/20 bytes occupied, 2 left. 0 available for the next buffer
  ret = framed_ring_buffer_put(&cbuf, src, 4);
  available = framed_ring_buffer_available(&cbuf);
  ok(ret == 0, "expected 0, got %d\n", ret);
  ok(available == 0, "expected 0 bytes available, got %lu", available);
  // try puting 1 byte : should fail
  ret = framed_ring_buffer_put(&cbuf, src, 1);
  ok(ret == CIR_ERROR_NO_SPACE_AVAILABLE, "expected CIR_ERROR_NO_SPACE_AVAILABLE, got %d\n", ret);


    // drop last buffer
  framed_ring_buffer_drop_last(&cbuf);
  // put 5 bytes. 12+7=19/20 bytes occupied, 0 left. 0 available for the next buffer
  ret = framed_ring_buffer_put(&cbuf, src, 5);
  available = framed_ring_buffer_available(&cbuf);
  ok(ret == 0, "expected 0, got %d\n", ret);
  ok(available == 0, "expected 0 bytes available, got %lu", available);
  // try puting 1 byte : should fail
  ret = framed_ring_buffer_put(&cbuf, src, 1);
  ok(ret == CIR_ERROR_NO_SPACE_AVAILABLE, "expected CIR_ERROR_NO_SPACE_AVAILABLE, got %d\n", ret);



  // get first buffer: 10 bytes
  ret = framed_ring_buffer_get(&cbuf, dst, sizeof(dst));
  ok(ret == 10, "expected %ld, got %d", 10, ret);
  ok(memcmp(dst, src, ret) == 0, "buffer corrupted");

  available = framed_ring_buffer_available(&cbuf);
  ok(available == 10, "expected 10 bytes available, got %lu", available);

  // put 10 bytes. len should wrap around the buffer. len: 1 byte at the end, 1 at the start, 
  ret = framed_ring_buffer_put(&cbuf, src, 10);
  ok(ret == 0, "expected 0, got %d\n", ret);



  // get 5 bytes buffer
  ret = framed_ring_buffer_get(&cbuf, dst, sizeof(dst));
  ok(ret == 5, "expected %ld, got %d", 5, ret);
  ok(memcmp(dst, src, ret) == 0, "buffer corrupted");

  // get 10 bytes buffer
  ret = framed_ring_buffer_get(&cbuf, dst, sizeof(dst));
  ok(ret == 10, "expected %ld, got %d", 10, ret);
  ok(memcmp(dst, src, ret) == 0, "buffer corrupted");

  // put 12 bytes buffer, data will wrap around
  // write_offset at 11, 2lenght+7data + 5data wrapped
  ret = framed_ring_buffer_put(&cbuf, src, 12);
  ok(ret == 0, "expected 0, got %d\n", ret);

  // get 12 bytes buffer
  ret = framed_ring_buffer_get(&cbuf, dst, sizeof(dst));
  ok(ret == 12, "expected %ld, got %d", 12, ret);
  ok(memcmp(dst, src, ret) == 0, "buffer corrupted");

  return 0;
}
