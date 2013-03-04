
#include "subsystems/sensors/baro.h"
#include <libopencm3/stm32/f1/gpio.h>

struct Baro baro;
struct BaroBoard baro_board;
struct i2c_transaction baro_trans;
struct bmp085_baro_calibration calibration;

#define BMP085_SAMPLE_PERIOD_MS (3 + (2 << BMP085_OSS) * 3)
#define BMP085_SAMPLE_PERIOD (BMP075_SAMPLE_PERIOD_MS >> 1)

// FIXME: BARO DRDY connected to PB0 for lisa/m

static inline void bmp085_write_reg(uint8_t addr, uint8_t value)
{
  baro_trans.buf[0] = addr;
  baro_trans.buf[1] = value;

  i2c_transmit(&i2c2, &baro_trans, BMP085_ADDR, 2);

  // FIXME, no while loops without timeout!!
  while (baro_trans.status == I2CTransPending || baro_trans.status == I2CTransRunning);
}

static inline void bmp085_read_reg16(uint8_t addr)
{
  baro_trans.buf[0] = addr;
  i2c_transceive(&i2c2, &baro_trans, BMP085_ADDR, 1, 2);
}

static inline int16_t bmp085_read_reg16_blocking(uint8_t addr, uint32_t timeout)
{
  uint32_t time = 0;

  bmp085_read_reg16(addr);

  while (baro_trans.status == I2CTransPending || baro_trans.status == I2CTransRunning) {
	  if ((time == timeout) && (timeout != 0)) {
		  return 0; /* Timeout of the i2c read */
	  } else {
		  time++;
	  }
  }

  return ((baro_trans.buf[0] << 8) | baro_trans.buf[1]);
}

static inline void bmp085_read_reg24(uint8_t addr)
{
  baro_trans.buf[0] = addr;
  i2c_transceive(&i2c2, &baro_trans, BMP085_ADDR, 1, 3);
}

static void bmp085_baro_read_calibration(void)
{
  calibration.ac1 = bmp085_read_reg16_blocking(0xAA, 10000); // AC1
  calibration.ac2 = bmp085_read_reg16_blocking(0xAC, 10000); // AC2
  calibration.ac3 = bmp085_read_reg16_blocking(0xAE, 10000); // AC3
  calibration.ac4 = bmp085_read_reg16_blocking(0xB0, 10000); // AC4
  calibration.ac5 = bmp085_read_reg16_blocking(0xB2, 10000); // AC5
  calibration.ac6 = bmp085_read_reg16_blocking(0xB4, 10000); // AC6
  calibration.b1 = bmp085_read_reg16_blocking(0xB6, 10000); // B1
  calibration.b2 = bmp085_read_reg16_blocking(0xB8, 10000); // B2
  calibration.mb = bmp085_read_reg16_blocking(0xBA, 10000); // MB
  calibration.mc = bmp085_read_reg16_blocking(0xBC, 10000); // MC
  calibration.md = bmp085_read_reg16_blocking(0xBE, 10000); // MD
}

void baro_init(void) {
  baro.status = BS_UNINITIALIZED;
  baro.absolute     = 0;
  baro.differential = 0;
  baro_board.status = LBS_UNINITIALIZED;
  bmp085_baro_read_calibration();

  /* STM32 specific (maybe this is a LISA/M specific driver anyway?) */
  gpio_clear(GPIOB, GPIO0);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
	        GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
}

static inline int baro_eoc(void)
{
  return gpio_get(GPIOB, GPIO0);
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

// Apply temp calibration and sensor calibration to raw measurement to get Pa (from BMP085 datasheet)
static int32_t baro_apply_calibration(int32_t raw)
{
  int32_t b6 = calibration.b5 - 4000;
  int x1 = (calibration.b2 * (b6 * b6 >> 12)) >> 11;
  int x2 = calibration.ac2 * b6 >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = (((calibration.ac1 * 4 + x3) << BMP085_OSS) + 2)/4;
  x1 = calibration.ac3 * b6 >> 13;
  x2 = (calibration.b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = (calibration.ac4 * (uint32_t) (x3 + 32768)) >> 15;
  uint32_t b7 = (raw - b3) * (50000 >> BMP085_OSS);
  int32_t p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  return p + ((x1 + x2 + 3791) >> 4);
}

void baro_event(void (*b_abs_handler)(void), void (*b_diff_handler)(void))
{
  if (baro_board.status == LBS_READING &&
      baro_trans.status != I2CTransPending && baro_trans.status != I2CTransRunning) {
    baro_board.status = LBS_REQUEST_TEMP;
    if (baro_trans.status == I2CTransSuccess) {
      int32_t tmp = (baro_trans.buf[0]<<16) | (baro_trans.buf[1] << 8) | baro_trans.buf[2];
      tmp = tmp >> (8 - BMP085_OSS);
      baro.absolute = baro_apply_calibration(tmp);
      b_abs_handler();
    }
  }
  if (baro_board.status == LBS_READING_TEMP &&
      baro_trans.status != I2CTransPending && baro_trans.status != I2CTransRunning) {
    baro_board.status = LBS_REQUEST;
    if (baro_trans.status == I2CTransSuccess) {
      // abuse differential to store temp in 0.1C for now
      int32_t tmp = (baro_trans.buf[0] << 8) | baro_trans.buf[1];
      int32_t x1 = ((tmp - calibration.ac6) * calibration.ac5) >> 15;
      int32_t x2 = (calibration.mc << 11) / (x1 + calibration.md);
      calibration.b5 = x1 + x2;
      baro.differential = (calibration.b5 + 8) >> 4;
      b_diff_handler();
    }
  }
}
