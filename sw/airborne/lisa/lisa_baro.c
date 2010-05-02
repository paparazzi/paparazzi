#include "lisa_baro.h"

#include "i2c.h"

static inline void baro_write_to_register(uint8_t baro_addr, uint8_t reg_addr, uint8_t val_msb, uint8_t val_lsb);
static inline void baro_read_from_register(uint8_t baro_addr, uint8_t reg_addr);
static inline void baro_set_current_register(uint8_t baro_addr, uint8_t reg_addr);
static inline void baro_read_from_current_register(uint8_t baro_addr);

struct LisaBaro baro;
// absolute
#define BARO_ABS_ADDR  0x90   
// differential
#define BARO_DIFF_ADDR 0x92

void baro_init(void) {
  baro.status = LBS_UNINITIALIZED;
}

void baro_periodic(void) {
  // check i2c_done
  switch (baro.status) {
  case LBS_UNINITIALIZED:
    baro_send_reset();
    baro.status = LBS_RESETED;
    break;
  case LBS_RESETED:
    baro_send_config_abs();
    baro.status = LBS_INITIALIZING_ABS;
    break;
  case LBS_INITIALIZING_ABS:
    baro_set_current_register(BARO_ABS_ADDR, 0x00);
    baro.status = LBS_INITIALIZING_ABS_1;
    break;
  case LBS_INITIALIZING_ABS_1:
    baro_send_config_diff();
    baro.status = LBS_INITIALIZING_DIFF;
    break;
  case LBS_INITIALIZING_DIFF:
    baro_set_current_register(BARO_DIFF_ADDR, 0x00);
    baro.status = LBS_INITIALIZING_DIFF_1;
    break;
  case LBS_INITIALIZING_DIFF_1:
  case LBS_READ_DIFF:
    baro_read_from_current_register(BARO_ABS_ADDR);
    baro.status = LBS_READING_ABS;
    break;
  case LBS_READ_ABS:
    baro_read_from_current_register(BARO_DIFF_ADDR);
    baro.status = LBS_READING_DIFF;
    break;
  default:
    break;
  }

}

void baro_send_reset(void) {
  i2c2.buf[0] = 0x06;
  i2c2_transmit(0x00, 1, &baro.i2c_done);
}

void baro_send_config_abs(void) {
  baro_write_to_register(BARO_ABS_ADDR, 0x01, 0x86, 0x83);
}

void baro_send_config_diff(void) {
  baro_write_to_register(BARO_DIFF_ADDR, 0x01, 0x84, 0x83);
}

static inline void baro_write_to_register(uint8_t baro_addr, uint8_t reg_addr, uint8_t val_msb, uint8_t val_lsb) {
  i2c2.buf[0] = reg_addr;
  i2c2.buf[1] = val_msb;
  i2c2.buf[2] = val_lsb;
  i2c2_transmit(baro_addr, 3, &baro.i2c_done);
}

static inline void baro_read_from_register(uint8_t baro_addr, uint8_t reg_addr) {
  i2c2.buf[0] = reg_addr;
  i2c2_transceive(baro_addr, 1, 2, &baro.i2c_done);
}

static inline void baro_set_current_register(uint8_t baro_addr, uint8_t reg_addr) {
  i2c2.buf[0] = reg_addr;
  i2c2_transmit(baro_addr, 1, &baro.i2c_done);
}

static inline void baro_read_from_current_register(uint8_t baro_addr) {
  i2c2_receive(baro_addr, 2, &baro.i2c_done);
}
