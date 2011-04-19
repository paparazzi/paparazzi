
#include "subsystems/sensors/baro.h"
#include <stm32/gpio.h>

struct Baro baro;
struct BaroBoard baro_board;
struct i2c_transaction baro_trans;
struct bmp085_baro_calibration calibration;

#define BMP085_SAMPLE_PERIOD_MS (3 + (2 << BMP085_OSS) * 3)
#define BMP085_SAMPLE_PERIOD (BMP075_SAMPLE_PERIOD_MS >> 1)

// FIXME: BARO DRDY connected to PB0 for lisa/m

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

static inline void bmp085_read_reg16(uint8_t addr)
{
  baro_trans.type = I2CTransTxRx;
  baro_trans.slave_addr = BMP085_ADDR;
  baro_trans.len_w = 1;
  baro_trans.len_r = 2;
  baro_trans.buf[0] = addr;
  i2c_submit(&i2c2, &baro_trans);
}

static inline int16_t bmp085_read_reg16_blocking(uint8_t addr)
{
  bmp085_read_reg16(addr);

  while (baro_trans.status == I2CTransPending || baro_trans.status == I2CTransRunning);

  return ((baro_trans.buf[0] << 8) | baro_trans.buf[1]);
}

static inline void bmp085_read_reg24(uint8_t addr)
{
  baro_trans.type = I2CTransTxRx;
  baro_trans.slave_addr = BMP085_ADDR;
  baro_trans.len_w = 1;
  baro_trans.len_r = 3;
  baro_trans.buf[0] = addr;
  i2c_submit(&i2c2, &baro_trans);
}

static void bmp085_baro_read_calibration(void)
{
  calibration.ac1 = bmp085_read_reg16_blocking(0xAA); // AC1
  calibration.ac2 = bmp085_read_reg16_blocking(0xAC); // AC2
  calibration.ac3 = bmp085_read_reg16_blocking(0xAE); // AC3
  calibration.ac4 = bmp085_read_reg16_blocking(0xB0); // AC4
  calibration.ac5 = bmp085_read_reg16_blocking(0xB2); // AC5
  calibration.ac6 = bmp085_read_reg16_blocking(0xB4); // AC6
  calibration.b1 = bmp085_read_reg16_blocking(0xB6); // B1
  calibration.b2 = bmp085_read_reg16_blocking(0xB8); // B2
  calibration.mb = bmp085_read_reg16_blocking(0xBA); // MB
  calibration.mc = bmp085_read_reg16_blocking(0xBC); // MC
  calibration.md = bmp085_read_reg16_blocking(0xBE); // MD
}

void baro_init(void) {
  baro.status = BS_UNINITIALIZED;
  baro.absolute     = 0;
  baro.differential = 0;
  baro_board.status = LBS_UNINITIALIZED;
  bmp085_baro_read_calibration();

  /* STM32 specific (maybe this is a LISA/M specific driver anyway?) */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static inline int baro_eoc(void)
{
  return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
}

static inline void bmp085_request_pressure(void)
{
  bmp085_write_reg(0xF4, 0x34 + (BMP085_OSS << 6));
}

static inline void bmp085_request_temp(void)
{
  bmp085_write_reg(0xF4, 0x2E);
}

static inline void bmp085_read_pressure(void)
{
  bmp085_read_reg24(0xF6);
}

static inline void bmp085_read_temp(void)
{
  bmp085_read_reg16(0xF6);
}

void baro_periodic(void) {
  // check that nothing is in progress
  if (baro_trans.status == I2CTransPending) return;
  if (baro_trans.status == I2CTransRunning) return;
  if (!i2c_idle(&i2c2)) return;

  switch (baro_board.status) {
  case LBS_UNINITIALIZED:
    baro_board_send_reset();
    baro_board.status = LBS_REQUEST;
    baro.status = BS_RUNNING;
    break;
  case LBS_REQUEST:
    bmp085_request_pressure();
    baro_board.status = LBS_READ;
    break;
  case LBS_READ:
    if (baro_eoc()) {
      bmp085_read_pressure();
      baro_board.status = LBS_READING;
    }
    break;
  case LBS_REQUEST_TEMP:
    bmp085_request_temp();
    baro_board.status = LBS_READ_TEMP;
    break;
  case LBS_READ_TEMP:
    if (baro_eoc()) {
      bmp085_read_temp();
      baro_board.status = LBS_READING_TEMP;
    }
    break;
  default:
    break;
  }

}

void baro_board_send_reset(void) {
  // This is a NOP at the moment
}
