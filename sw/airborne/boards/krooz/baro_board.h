
/*
 * board specific fonctions for the lisa_m board
 *
 */

#ifndef BOARDS_KROOZ_BARO_H
#define BOARDS_KROOZ_BARO_H



#include "std.h"
#include "mcu_periph/i2c.h"

//#include "led.h"

#ifndef MS5611_I2C_DEV
#define MS5611_I2C_DEV i2c2
#endif

/* address can be 0xEC or 0xEE (CSB\ high = 0xEC) */
#define MS5611_SLAVE_ADDR 0xEC

/* we use OSR=4096 for maximum resolution */
#define MS5611_SOFT_RESET       0x1E
#define MS5611_PROM_READ        0xA0
#define MS5611_START_CONV_D1    0x48
#define MS5611_START_CONV_D2    0x58
#define MS5611_ADC_READ         0x00

#define PROM_NB                 8

#define PAUSE_CNT 5

enum KroozBaroStatus {
  MS5611_UNINIT,
  MS5611_RESET,
  MS5611_RESET_OK,
  MS5611_PROM,
  MS5611_IDLE,
  MS5611_CONV_D1,
  MS5611_CONV_D1_OK,
  MS5611_ADC_D1,
  MS5611_CONV_D2,
  MS5611_CONV_D2_OK,
  MS5611_ADC_D2
};

struct BaroBoard {
  enum KroozBaroStatus status;
};

extern struct BaroBoard baro_board;
extern struct i2c_transaction baro_trans;
extern volatile uint8_t baro_status;
extern uint16_t baro_c[PROM_NB];
extern uint32_t baro_d1, baro_d2;
extern int32_t prom_cnt;

extern uint8_t pause_cnt;

extern void baro_ms5611_d1(void);
extern void baro_ms5611_d2(void);

static inline int8_t baro_ms5611_crc(uint16_t* prom) {
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

static inline bool_t baro_read_reg(uint8_t addr, uint8_t len_w, uint8_t len_r)
{
  baro_trans.type = I2CTransTxRx;
  baro_trans.slave_addr = MS5611_SLAVE_ADDR;
  baro_trans.len_w = len_w;
  baro_trans.len_r = len_r;
  baro_trans.buf[0] = addr;
  if(i2c_submit(&MS5611_I2C_DEV, &baro_trans))
		return TRUE;
	return FALSE;
}

static inline void baro_event(void (*b_abs_handler)(void), void (*b_diff_handler)(void))
{
  if (baro_trans.status == I2CTransSuccess) {
	
	//RunOnceEvery(50,LED_TOGGLE(3));
	
    switch (baro_status) {

    case MS5611_RESET:
      baro_status = MS5611_RESET_OK;
      baro_trans.status = I2CTransDone;
      break;

    case MS5611_PROM:
      /* read prom data */
      baro_c[prom_cnt++] = (baro_trans.buf[0] << 8) | baro_trans.buf[1];
      if (prom_cnt < PROM_NB) {
        /* get next prom data */
        baro_trans.buf[0] = MS5611_PROM_READ | (prom_cnt << 1);
        i2c_transceive(&MS5611_I2C_DEV, &baro_trans, MS5611_SLAVE_ADDR, 1, 3);
      }
      else {
        /* done reading prom */
        baro_trans.status = I2CTransDone;
        /* check prom crc */
        if (baro_ms5611_crc(baro_c) == 0) {
          baro_status = MS5611_IDLE;
					baro.status = BS_RUNNING;
        }
        else {
          /* checksum error, try again */
          baro_status = MS5611_UNINIT;
					prom_cnt = 0;
        }
      }
      break;

    case MS5611_CONV_D1:
		  /*baro_trans.status = I2CTransDone;
		  baro_status = MS5611_IDLE;
		  if(--pause_cnt == 0) 
			{
				if(baro_read_reg(MS5611_ADC_READ, 1, 4))
				  baro_status = MS5611_ADC_D1;
				else
					pause_cnt = 1;
			}*/
      break;

    case MS5611_ADC_D1:
		  /*baro_status = MS5611_IDLE;
			baro_trans.status = I2CTransDone;
			baro.absolute = (baro_trans.buf[0] << 16) |
                  (baro_trans.buf[1] << 8) |
                  baro_trans.buf[2];
		
       read D1 (pressure) */
      baro_d1 = (baro_trans.buf[0] << 16) |
                  (baro_trans.buf[1] << 8) |
                  baro_trans.buf[2];
			
      /* start D2 conversion */
      baro_status = MS5611_CONV_D2;
      baro_trans.buf[0] = MS5611_START_CONV_D2;
      i2c_transmit(&MS5611_I2C_DEV, &baro_trans, MS5611_SLAVE_ADDR, 1);
			pause_cnt = PAUSE_CNT;
      break;

    case MS5611_CONV_D2:
		/*
		  baro_trans.status = I2CTransDone;
		  if(!--pause_cnt) 
			{
			  baro_status = MS5611_ADC_D2;
        baro_trans.buf[0] = MS5611_ADC_READ;
        i2c_transceive(&MS5611_I2C_DEV, &baro_trans, MS5611_SLAVE_ADDR, 1, 4);
			}
		*/
      break;

    case  MS5611_ADC_D2: {
      int64_t dt, tempms, off, sens, t2, off2, sens2;
      /* read D2 (temperature) */
      baro_d2 = (baro_trans.buf[0] << 16) |
                  (baro_trans.buf[1] << 8) |
                  baro_trans.buf[2];
      baro_status = MS5611_IDLE;
      baro_trans.status = I2CTransDone;

      /* difference between actual and ref temperature */
      dt = baro_d2 - (int64_t)baro_c[5] * (1<<8);
      /* actual temperature */
      tempms = 2000 + ((int64_t)dt * baro_c[6]) / (1<<23);
      /* offset at actual temperature */
      off = ((int64_t)baro_c[2] * (1<<16)) + ((int64_t)baro_c[4] * dt) / (1<<7);
      /* sensitivity at actual temperature */
      sens = ((int64_t)baro_c[1] * (1<<15)) + ((int64_t)baro_c[3] * dt) / (1<<8);
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
      baro.absolute = (((int64_t)baro_d1 * sens) / (1<<21) - off) / (1<<15);
#ifdef SENSOR_SYNC_SEND
      ftempms = tempms / 100.;
      fbaroms = baroms / 100.;
      DOWNLINK_SEND_BARO_MS5611(DefaultChannel, DefaultDevice,
                                &baro_d1, &baro_d2, &fbaroms, &ftempms);
#endif
			b_abs_handler();
      break;
    }

    default:
      baro_trans.status = I2CTransDone;
      break;
    }
  }
	if (baro_trans.status == I2CTransFailed) {
	  baro_trans.status = I2CTransDone;
		baro_status = MS5611_IDLE;
	}
}

#define BaroEvent(_b_abs_handler, _b_diff_handler) baro_event(_b_abs_handler,_b_diff_handler)

#endif /* BOARDS_LISA_M_BARO_H */
