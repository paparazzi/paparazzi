/*
 * General purpose circular buffer
 *
 * Copyright (C) 2021 Fabien-B <fabien-b@github.com> 
 *
 * This file is part of paparazzi. See LICENCE file.
 */

#include "circular_buffer.h"
#include <string.h>


void circular_buffer_init(struct circular_buffer *cb, uint8_t *buffer, size_t len)
{
  cb->_buf = buffer;
  cb->_buf_len = len;
  cb->read_offset = 0;
  cb->write_offset = 0;
}


int circular_buffer_get(struct circular_buffer *cb, uint8_t *buf, size_t len)
{
  // buffer empty
  if (cb->read_offset == cb->write_offset) { return CIR_ERROR_NO_MSG; }
  // LEN| MSG...| LEN | MSG...
  uint16_t msg_len_p = cb->_buf[cb->read_offset] | (cb->_buf[(cb->read_offset + 1) % cb->_buf_len] << 8);
  // output buffer too small
  if (len < msg_len_p) { return CIR_ERROR_BUFFER_TOO_SMALL; }

  size_t end_offset = cb->read_offset + msg_len_p + 2;
  if (end_offset >= cb->_buf_len) {
    end_offset -= cb->_buf_len;
  }
  uint8_t *start = cb->_buf + ((cb->read_offset + 2) % cb->_buf_len);

  // No wrapping can be read in one go
  if (end_offset >= ((cb->read_offset + 2) % cb->_buf_len)) {
    memcpy(buf, start, msg_len_p);
  } else {
    size_t len1 = cb->_buf_len - (cb->read_offset + 2);
    size_t len2 = len - len1;
    memcpy(buf, start, len1);
    memcpy(buf + len1, cb->_buf, len2);
  }

  int nb_bytes = msg_len_p;
  cb->read_offset = end_offset;
  return nb_bytes;
}

int circular_buffer_put(struct circular_buffer *cb, const uint8_t *buf, size_t len)
{
  int available = 0;
  if (cb->read_offset > cb->write_offset) {
    available = cb->read_offset - cb->write_offset - 2;
  } else {
    available = cb->_buf_len - (cb->write_offset - cb->read_offset) - 2;
  }

  /**
   * len == available is invalid because it will cause
   * write_offset to be equal to read_offset, which is considered an empty buffer.
  */
  if ((int)len >= available) {
    return CIR_ERROR_NO_SPACE_AVAILABLE;
  }

  size_t end_offset = cb->write_offset + len + 2;
  if (end_offset >= cb->_buf_len) {
    end_offset -= cb->_buf_len;
  }


  // Write the length and wrap around if needed
  cb->_buf[cb->write_offset] = len & 0xFF;
  cb->_buf[(cb->write_offset + 1) % cb->_buf_len] = (len >> 8) & 0xFF;

  // Write the date no wrapping needed
  if (end_offset > cb->write_offset) {
    memcpy(cb->_buf + cb->write_offset + 2, buf, len);
  } 
  // Wrapping data partially
  else if ((cb->write_offset + 2) < cb->_buf_len) {
    size_t len1 = cb->_buf_len - (cb->write_offset + 2);
    size_t len2 = len - len1;
    memcpy(cb->_buf + cb->write_offset + 2, buf, len1);
    memcpy(cb->_buf, buf + len1, len2);
  }
  // Only data is wrapped (starting at the beginning or 1 byte in for wrapped length)
  else {
    memcpy(&cb->_buf[(cb->write_offset + 2) % cb->_buf_len], buf, len);
  }

  cb->write_offset = end_offset;
  return 0;
}

int circular_buffer_drop(struct circular_buffer *cb) {
  // buffer empty
  if (cb->read_offset == cb->write_offset) { return CIR_ERROR_NO_MSG; }

  size_t record_head_offset = cb->read_offset;
  size_t end_offset = cb->read_offset;

  while(end_offset != cb->write_offset) {
    size_t record_head_offset = end_offset;
    uint16_t* msg_len_p = (uint16_t*)&cb->_buf[record_head_offset];
    size_t end_offset = record_head_offset + *msg_len_p + 2;
    if (end_offset >= cb->_buf_len) {
      end_offset -= cb->_buf_len;
    }
  }

  cb->write_offset = record_head_offset;
  return 0;
}
