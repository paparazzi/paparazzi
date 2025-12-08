/*
 * General purpose circular buffer
 *
 * Copyright (C) 2021 Fabien-B <fabien-b@github.com> 
 *
 * This file is part of paparazzi. See LICENCE file.
 */

#pragma once
#include <inttypes.h>
#include <stddef.h>


typedef struct {
  size_t read_offset;
  size_t write_offset;
  size_t size;
  uint8_t *buf;
} ring_buffer_t;


/**
 * Init @param ring_buffer with the @param buf subjacent buffer of size @param size.
 */
void ring_buffer_init (ring_buffer_t *ring_buffer, uint8_t *buf, size_t size);

/**
 * Write @param data of size @param len in @param ring_buffer.
 * @returns the number of byte effectively written in the ring_buffer.
 */
size_t ring_buffer_write(ring_buffer_t *ring_buffer, uint8_t* data, size_t len);

/**
 * Read @param len bytes from @param ring_buffer into @param read_buffer.
 * @returns the number of byte effectively read from the ring_buffer.
 */
size_t ring_buffer_read(ring_buffer_t *ring_buffer, uint8_t* read_buffer, size_t len);

/**
 * @returns the number of bytes available in the @param ring_buffer.
 */
size_t ring_buffer_available(ring_buffer_t *ring_buffer);

/**
 * @returns the number of bytes that can be written in @param ring_buffer
 */
size_t ring_buffer_free_space(ring_buffer_t *ring_buffer);

