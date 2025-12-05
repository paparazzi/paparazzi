/*
 * General purpose circular buffer
 *
 * Copyright (C) 2021 Fabien-B <fabien-b@github.com> 
 *
 * This file is part of paparazzi. See LICENCE file.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

/**
 * This is a general purpose circular buffer for storing variable lenght buffers in a FIFO order.
 * Buffers length are stored as uint16_t.
 * 
 * Declare a \ref framed_ring_buffer and allocate a buffer that will outlive it.
 * Initialize the \ref framed_ring_buffer using \ref framed_ring_buffer_init.
 * 
*/

struct framed_ring_buffer {
  size_t read_offset;
  size_t write_offset;
  size_t _buf_len;
  uint8_t *_buf;
};

enum cir_error {
  CIR_ERROR_NO_MSG = -1,                /**< circular buffer is empty */
  CIR_ERROR_BUFFER_TOO_SMALL = -2,      /**< destination buffer is too small */
  CIR_ERROR_NO_SPACE_AVAILABLE = -3,    /**< no space available in the circular buffer */
  CIR_ERROR_LOCKED = -4,                /**< mutex locked */
};

/**
 * @brief initialize a circular buffer.
 * @param cb framed_ring_buffer structure
 * @param buffer buffer used internally by the framed ring buffer
 * @param len size of \p buffer
*/
void framed_ring_buffer_init(struct framed_ring_buffer *cb, uint8_t *buffer, size_t len);


/**
 * @brief copy the next buffer available in \p cb to \p buf.
 * @param cb The circular buffer
 * @param buf destination buffer
 * @param len size of \p buf
 * @return Size of the data copied to \p buf, or an error code if negative.
*/
int framed_ring_buffer_get(struct framed_ring_buffer *cb, uint8_t *buf, size_t len);

/**
 * @brief Copy \p buf in the circular buffer
 * @param cb The circular buffer
 * @param buf source buffer
 * @param len Size of \p buf
 * @return 0 on success, Error code if negative
*/
int framed_ring_buffer_put(struct framed_ring_buffer *cb, const uint8_t *buf, size_t len);


/**
 * @brief Drop last inserted record
 */
int framed_ring_buffer_drop_last(struct framed_ring_buffer *cb);

/**
 * @brief Get the available sapce for the next buffer
 */
size_t framed_ring_buffer_available(struct framed_ring_buffer *cb);

/**
 * @brief Clear buffer
 */
void framed_ring_buffer_clear(struct framed_ring_buffer *cb);