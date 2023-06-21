#pragma once

#include <stdint.h>
#include <stddef.h>

struct cir_buf
{
  size_t read_offset;
  size_t write_offset;
  size_t _buf_len;
  uint8_t* _buf;
};

enum cir_error {
  CIR_ERROR_NO_MSG = -1,
  CIR_ERROR_BUFFER_TOO_SMALL = -2,
  CIR_MSG_TOO_LARGE = -3,
  CIR_ERROR_NO_SPACE_AVAILABLE = -4,
};

int cir_buf_get(struct cir_buf* rg, uint8_t* buf, size_t len);
int cir_buf_put(struct cir_buf* rg, uint8_t* buf, size_t len);
void cir_buf_init(struct cir_buf* rg, uint8_t* buffer, size_t len);
