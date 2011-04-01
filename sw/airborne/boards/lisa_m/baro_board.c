
#include "subsystems/sensors/baro.h"

struct Baro baro;
struct BaroBoard baro_board;
struct i2c_transaction baro_trans;


static inline void baro_board_write_to_register(uint8_t baro_addr, uint8_t reg_addr, uint8_t val_msb, uint8_t val_lsb);
static inline void baro_board_read_from_register(uint8_t baro_addr, uint8_t reg_addr);
static inline void baro_board_set_current_register(uint8_t baro_addr, uint8_t reg_addr);
static inline void baro_board_read(void);


#define BMP085_SAMPLE_PERIOD_MS (3 + (2 << BMP085_OSS) * 3)
#define BMP085_SAMPLE_PERIOD (BMP075_SAMPLE_PERIOD_MS >> 1)

void baro_init(void) {
  baro.status = BS_UNINITIALIZED;
  baro.absolute     = 0;
  baro.differential = 0;
  baro_board.status = LBS_UNINITIALIZED;
}

static inline void bmp085_write_reg(uint8_t addr, uint8_t value)
{
  baro_trans.type = I2CTransTx;
  baro_trans.slave_addr = BMP085_ADDR;
  baro_trans.len_w = 2;
  baro_trans.buf[0] = addr;
  baro_trans.buf[1] = value;
  i2c_submit(&i2c2, &baro_trans);
  while (baro_trans.status == I2CTransPending || baro_trans.status == I2CTransRunning);
}

static inline void bmp085_request_pressure(void)
{
  bmp085_write_reg(0xF4, 0x34 + (BMP085_OSS << 6));
}


void baro_periodic(void) {
  // check i2c_done
  if (!i2c_idle(&i2c2)) return;
  switch (baro_board.status) {
  case LBS_UNINITIALIZED:
    baro_board_send_reset();
    baro_board.status = LBS_RESETED;
    break;
  case LBS_RESETED:
    baro_board_send_config();
    baro_board.status = LBS_INITIALIZING;
    break;
  case LBS_INITIALIZING:
    baro_board_set_current_register(BMP085_ADDR, 0x00);
    baro_board.status = LBS_INITIALIZING_1;
    break;
  case LBS_INITIALIZING_1:
    baro.status = BS_RUNNING;
  case LBS_REQUEST:
    bmp085_request_pressure();
    baro_board.status = LBS_READ;
    break;
  case LBS_READ:
    baro_board_read();
    baro_board.status = LBS_READING;
    break;
  default:
    break;
  }

}


void baro_board_send_config(void) {
  /* maybe we should read factory calibration here */
  //baro_board_write_to_register(BMP085_ADDR, 0x01, 0x86, 0x83);
}

void baro_board_send_reset(void) {
  baro_trans.type = I2CTransTx;
  baro_trans.slave_addr = 0x00;
  baro_trans.len_w = 1;
  baro_trans.buf[0] = 0x06;
  i2c_submit(&i2c2,&baro_trans);
}

static inline void bmp085_read_reg24(uint8_t addr)
{
  baro_trans.type = I2CTransTxRx;
  baro_trans.slave_addr = BMP085_ADDR;
  baro_trans.len_w = 1;
  baro_trans.len_r = 3;
  baro_trans.buf[0] = addr;
  i2c_submit(&i2c2, &baro_trans);
  //while (baro_trans.status == I2CTransPending || baro_trans.status == I2CTransRunning);

  //return (baro_trans.buf[0] << 16) | (baro_trans.buf[1] >> 8) | (baro_trans.buf[2]);
}

static inline void baro_board_write_to_register(uint8_t baro_addr, uint8_t reg_addr, uint8_t val_msb, uint8_t val_lsb) {
  baro_trans.type = I2CTransTx;
  baro_trans.slave_addr = baro_addr;
  baro_trans.len_w = 3;
  baro_trans.buf[0] = reg_addr;
  baro_trans.buf[1] = val_msb;
  baro_trans.buf[2] = val_lsb;
  i2c_submit(&i2c2,&baro_trans);
}

static inline void baro_board_read_from_register(uint8_t baro_addr, uint8_t reg_addr) {
  baro_trans.type = I2CTransTxRx;
  baro_trans.slave_addr = baro_addr;
  baro_trans.len_w = 1;
  baro_trans.len_r = 2;
  baro_trans.buf[0] = reg_addr;
  i2c_submit(&i2c2,&baro_trans);
}

static inline void baro_board_set_current_register(uint8_t baro_addr, uint8_t reg_addr) {
  baro_trans.type = I2CTransTx;
  baro_trans.slave_addr = baro_addr;
  baro_trans.len_w = 1;
  baro_trans.buf[0] = reg_addr;
  i2c_submit(&i2c2,&baro_trans);
}


static inline void bmp085_read_pressure(void)
{
  bmp085_read_reg24(0xF6);
}

static inline void baro_board_read()
{
  //int32_t x;
  //bmp085_request_pressure();
  bmp085_read_pressure();
  //baro_trans.type = I2CTransRx;
  //baro_trans.slave_addr = BMP085_ADDR;
  //baro_trans.len_r = 2;
  //i2c_submit(&i2c2,&baro_trans);
}
