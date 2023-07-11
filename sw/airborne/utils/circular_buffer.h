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
 * This is a general purpose circular buffer for storing buffers in a FIFO order.
 * A current limitation: the size of the buffers is limited to 255, is size beeing stored on a uint8_t.
 * 
 * Declare a \ref circular_buffer and allocate a buffer that will outlive it.
 * Initialize the \ref circular_buffer using \ref circular_buffer_init.
 * 
*/

struct circular_buffer {
  size_t read_offset;
  size_t write_offset;
  size_t _buf_len;
  uint8_t *_buf;
};

enum cir_error {
  CIR_ERROR_NO_MSG = -1,                /**< circular buffer is empty */
  CIR_ERROR_BUFFER_TOO_SMALL = -2,      /**< destination buffer is too small */
  CIR_ERROR_NO_SPACE_AVAILABLE = -3,    /**< no space available in the circular buffer */
};

/**
 * @brief initialize a circular buffer.
 * @param cb circular_buffer structure
 * @param buffer buffer used internally by the circular buffer
 * @param len size of \p buffer
*/
void circular_buffer_init(struct circular_buffer *cb, uint8_t *buffer, size_t len);


/**
 * @brief copy the next buffer available in \p cb to \p buf.
 * @param cb The circular buffer
 * @param buf destination buffer
 * @param len size of \p buf
 * @return Size of the data copied to \p buf, or an error code if negative.
*/
int circular_buffer_get(struct circular_buffer *cb, uint8_t *buf, size_t len);

/**
 * @brief Copy \p buf in the circular buffer
 * @param cb The circular buffer
 * @param buf source buffer
 * @param len Size of \p buf
 * @return 0 on success, Error code if negative
*/
int circular_buffer_put(struct circular_buffer *cb, uint8_t *buf, size_t len);
