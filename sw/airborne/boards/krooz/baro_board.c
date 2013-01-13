
#include "subsystems/sensors/baro.h"
/*
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
*/
struct Baro baro;
struct BaroBoard baro_board;
struct i2c_transaction baro_trans;

volatile uint8_t baro_status;
uint16_t baro_c[PROM_NB];
uint32_t baro_d1, baro_d2;
int32_t prom_cnt;
float fbaroms, ftempms;

//uint32_t time = 0;
uint8_t pause_cnt = 0;

void baro_init(void) {
  baro.status = BS_UNINITIALIZED;
  baro_status = MS5611_UNINIT;
  prom_cnt = 0;
}

void baro_periodic(void) {
/*
	uint8_t buf[2];
	buf[0] = baro_trans.status;
	buf[1] = baro_status;
	RunOnceEvery(20,DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, 2, buf));
*/
	{
    if (baro_status == MS5611_IDLE) {
      /* start D1 conversion */
      baro_status = MS5611_CONV_D1;
      baro_trans.buf[0] = MS5611_START_CONV_D1;
      I2CTransmit(MS5611_I2C_DEV, baro_trans, MS5611_SLAVE_ADDR, 1);
			
			pause_cnt = PAUSE_CNT;
			//RunOnceEvery(10,LED_TOGGLE(2));
    }
		else if (baro_status == MS5611_CONV_D1) {
		  if(baro_read_reg(MS5611_ADC_READ, 1, 3))
				  baro_status = MS5611_ADC_D1;
		}
		else if (baro_status == MS5611_CONV_D2) {
		  if(baro_read_reg(MS5611_ADC_READ, 1, 3))
				  baro_status = MS5611_ADC_D2;
		}
    else if (baro_status == MS5611_UNINIT) {
      /* reset sensor */
      baro_status = MS5611_RESET;
      baro_trans.buf[0] = MS5611_SOFT_RESET;
      I2CTransmit(MS5611_I2C_DEV, baro_trans, MS5611_SLAVE_ADDR, 1);
    }
    else if (baro_status == MS5611_RESET_OK) {
      /* start getting prom data */
      baro_status = MS5611_PROM;
      baro_trans.buf[0] = MS5611_PROM_READ | (prom_cnt << 1);
      I2CTransceive(MS5611_I2C_DEV, baro_trans, MS5611_SLAVE_ADDR, 1, 3);
    }
  }
}

