/**
 *  Measurement Specialties (Intersema) MS5611-01BA pressure/temperature sensor interface for I2C
 *
 * Edit by: Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
 * Utah State University, http://aggieair.usu.edu/
 */
#include "subsystems/sensors/baro.h"
#include "baro_board.h"
#include "peripherals/ms5611.h"
#include "led.h"
#include "std.h"
#include "mcu_periph/sys_time.h"

#include "mcu_periph/spi.h"
#ifndef MS5611_SPI_DEV
#define MS5611_SPI_DEV spi2
#endif
#define MS5611_BUFFER_LENGTH    4

#ifdef DEBUG
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#endif

struct Baro baro;
struct spi_transaction ms5611_trans;
uint8_t ms5611_status;
int32_t prom_cnt;
uint16_t ms5611_c[PROM_NB];
uint32_t ms5611_d1, ms5611_d2;
float fbaroms, ftempms;
volatile uint8_t input_buf_ms5611[MS5611_BUFFER_LENGTH];
volatile uint8_t output_buf_ms5611[MS5611_BUFFER_LENGTH];
static void trans_cb_ms5611( struct spi_transaction *trans );

static int8_t baro_ms5611_crc(uint16_t* prom) {
  int32_t i, j;
  uint32_t res = 0;
  uint8_t crc = prom[7] & 0xF;
  prom[7] &= 0xFF00;
  for (i = 0; i < 16; i++) {
    if (i & 1) res ^= ((prom[i>>1]) & 0x00FF);
    else res ^= (prom[i>>1]>>8);
    for (j = 8; j > 0; j--) {
      if (res & 0x8000) res ^= 0x1800;
      res <<= 1;
    }
  }
  prom[7] |= crc;
  if (crc == ((res >> 12) & 0xF)) return 0;
  else return -1;
}

static void trans_cb_ms5611( struct spi_transaction *trans ) {
#ifdef ROTORCRAFT_BARO_LED
  RunOnceEvery(10,LED_TOGGLE(ROTORCRAFT_BARO_LED));
#endif
}

void baro_init(void) {
  ms5611_trans.select = SPISelectUnselect;
  ms5611_trans.cpol = SPICpolIdleHigh;
  ms5611_trans.cpha = SPICphaEdge2;
  ms5611_trans.dss = SPIDss8bit;
  ms5611_trans.bitorder = SPIMSBFirst;
  ms5611_trans.cdiv = SPIDiv64;
  ms5611_trans.slave_idx = MS5611_SLAVE_DEV;
  ms5611_trans.output_length = MS5611_BUFFER_LENGTH;
  ms5611_trans.input_length = MS5611_BUFFER_LENGTH;
  ms5611_trans.after_cb = trans_cb_ms5611;
  ms5611_trans.input_buf = &input_buf_ms5611[0];
  ms5611_trans.output_buf = &input_buf_ms5611[0];

  ms5611_status = MS5611_UNINIT;
  baro.status = BS_UNINITIALIZED;
  prom_cnt = 0;
}

void baro_periodic(void) {
  if (cpu_time_sec > 1) {
    if (ms5611_status == MS5611_IDLE) {
      /* start D1 conversion */
      ms5611_status = MS5611_CONV_D1;
      ms5611_trans.output_buf[0] = MS5611_START_CONV_D1;
      spi_submit(&(MS5611_SPI_DEV), &ms5611_trans);
    #ifdef DEBUG
    RunOnceEvery(300, { DOWNLINK_SEND_MS5611_COEFF(DefaultChannel, DefaultDevice,
              &ms5611_c[0], &ms5611_c[1], &ms5611_c[2], &ms5611_c[3],
              &ms5611_c[4], &ms5611_c[5], &ms5611_c[6], &ms5611_c[7]);});
    #endif
    }
    else if (ms5611_status == MS5611_CONV_D1) {
    /* assume D1 conversion is done */
      ms5611_status = MS5611_CONV_D1_OK;
    }
    else if (ms5611_status == MS5611_CONV_D1_OK) {
      /* read D1 adc */
      ms5611_status = MS5611_ADC_D1;
      ms5611_trans.output_buf[0] = MS5611_ADC_READ;
      spi_submit(&(MS5611_SPI_DEV), &ms5611_trans);
    }
    else if (ms5611_status == MS5611_CONV_D2) {
    /* assume D2 conversion is done */
      ms5611_status = MS5611_CONV_D2_OK;
    }
    else if (ms5611_status == MS5611_CONV_D2_OK) {
      /* read D2 adc */
      ms5611_status = MS5611_ADC_D2;
      ms5611_trans.output_buf[0] = MS5611_ADC_READ;
      spi_submit(&(MS5611_SPI_DEV), &ms5611_trans);
    }
    else if (ms5611_status == MS5611_UNINIT) {
      /* reset sensor */
      ms5611_status = MS5611_RESET;
      ms5611_trans.output_buf[0] = MS5611_SOFT_RESET;
      spi_submit(&(MS5611_SPI_DEV), &ms5611_trans);
    }
    else if (ms5611_status == MS5611_RESET_OK) {
      /* start getting prom data */
      ms5611_status = MS5611_PROM;
      ms5611_trans.output_buf[0] = MS5611_PROM_READ | (prom_cnt << 1);
      spi_submit(&(MS5611_SPI_DEV), &ms5611_trans);
    }
  }
}

void baro_event(void (*b_abs_handler)(void), void (*b_diff_handler)(void)){
  if (ms5611_trans.status == SPITransSuccess) {
    switch (ms5611_status) {

    case MS5611_RESET:
      ms5611_status = MS5611_RESET_OK;
      ms5611_trans.status = SPITransDone;
      break;

    case MS5611_PROM:
      /* read prom data */
      ms5611_c[prom_cnt++] = (ms5611_trans.input_buf[1] << 8) | ms5611_trans.input_buf[2];
      if (prom_cnt < PROM_NB) {//8 bytes at PROM
        /* get next prom data */
        ms5611_trans.output_buf[0] = MS5611_PROM_READ | (prom_cnt << 1);
        spi_submit(&(MS5611_SPI_DEV), &ms5611_trans);
      }
      else {
        /* done reading prom */
        ms5611_trans.status = SPITransDone;
        /* check prom crc */
        if (baro_ms5611_crc(ms5611_c) == 0) {
          ms5611_status = MS5611_IDLE;
          baro.status = BS_RUNNING;
        }
        else {
          /* checksum error, try again */
          baro_init();
        }
      }
      break;

    case  MS5611_ADC_D1:
      /* read D1 (pressure) */
      ms5611_d1 = (ms5611_trans.input_buf[1] << 16) |
                  (ms5611_trans.input_buf[2] << 8) |
                  ms5611_trans.input_buf[3];
      /* start D2 conversion */
      ms5611_status = MS5611_CONV_D2;
      ms5611_trans.output_buf[0] = MS5611_START_CONV_D2;
      spi_submit(&(MS5611_SPI_DEV), &ms5611_trans);
      break;

    case  MS5611_ADC_D2: {
      int64_t dt, baroms, tempms, off, sens, t2, off2, sens2;
      /* read D2 (temperature) */
      ms5611_d2 = (ms5611_trans.input_buf[1] << 16) |
                  (ms5611_trans.input_buf[2] << 8) |
                  ms5611_trans.input_buf[3];
      ms5611_status = MS5611_IDLE;
      ms5611_trans.status = SPITransDone;

      /* difference between actual and ref temperature */
      dt = ms5611_d2 - (int64_t)ms5611_c[5] * (1<<8);
      /* actual temperature */
      tempms = 2000 + ((int64_t)dt * ms5611_c[6]) / (1<<23);
      /* offset at actual temperature */
      off = ((int64_t)ms5611_c[2] * (1<<16)) + ((int64_t)ms5611_c[4] * dt) / (1<<7);
      /* sensitivity at actual temperature */
      sens = ((int64_t)ms5611_c[1] * (1<<15)) + ((int64_t)ms5611_c[3] * dt) / (1<<8);
      /* second order temperature compensation */
      if (tempms < 2000) {
        t2 = (dt*dt) / (1<<31);
        off2 = 5 * ((int64_t)(tempms-2000)*(tempms-2000)) / (1<<1);
        sens2 = 5 * ((int64_t)(tempms-2000)*(tempms-2000)) / (1<<2);
        if (tempms < -1500) {
          off2 = off2 + 7 * (int64_t)(tempms+1500)*(tempms+1500);
          sens2 = sens2 + 11 * ((int64_t)(tempms+1500)*(tempms+1500)) / (1<<1);
        }
        tempms = tempms - t2;
        off = off - off2;
        sens = sens - sens2;
      }
      /* temperature compensated pressure */
      baroms = (((int64_t)ms5611_d1 * sens) / (1<<21) - off) / (1<<15);

      /* Update baro structure */
      baro.absolute = (int32_t)baroms;

      b_abs_handler();
      b_diff_handler();

      #ifdef DEBUG
      ftempms = tempms / 100.;
      fbaroms = baroms / 100.;
      DOWNLINK_SEND_BARO_MS5611(DefaultChannel, DefaultDevice,
                                 &ms5611_d1, &ms5611_d2, &fbaroms, &ftempms, &baro.status);
      #endif
      break;
    }

    default:
      ms5611_trans.status = SPITransDone;
      break;
    }
  }
}
