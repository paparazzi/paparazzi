#include "tl_baro.h"

#include "airframe.h"

#define CMD_MEASUREMENT 0x0F
#define CMD_PRESSURE    0x40
#define CMD_TEMPERATURE 0x20


uint16_t tl_baro_d[TL_BARO_NB_DATA];
const uint8_t tl_baro_cmd[TL_BARO_NB_DATA] = {CMD_PRESSURE, CMD_TEMPERATURE};
uint32_t tl_baro_pressure;
uint16_t tl_baro_temp;
uint8_t tl_baro_cur_data;

static uint16_t c1, c2, c3, c4, ut1, c6;

#define MS5534A_MCLK 32768
#define PWM_PRESCALER 1
#define PWM_PERIOD ((PCLK / PWM_PRESCALER) / MS5534A_MCLK)
#define PWM_DUTY (PWM_PERIOD / 2)


void tl_baro_init(void) {
  
  /* 32768Hz on PWM2 */
  /* Configure P0.7 pin as PWM */
  PINSEL0 &= ~(_BV(14));
  PINSEL0 |= _BV(15);

  /* No prescaler */
  PWMPR = PWM_PRESCALER-1;

  /* To get 32768Hz from a base frequency of 15MHz */
  PWMMR0 = PWM_PERIOD;
  PWMMR2 = PWM_DUTY;

  PWMLER = PWMLER_LATCH0 | PWMLER_LATCH2;
  PWMTCR = PWMTCR_COUNTER_ENABLE | PWMTCR_PWM_ENABLE;
  PWMPCR = PWMPCR_ENA2;

  /* setup slave select */
  /* configure SS pin */
  SetBit(TL_BARO_SS_IODIR, TL_BARO_SS_PIN); /* pin is output  */
  TlBaroUnselect();                         /* pin idles high */

  tl_baro_cur_data = TL_BARO_PRESSURE; 


  /* End of init, configuration (page 11) */
  c1 = TL_BARO_W1 >> 1;
  c2 = ((TL_BARO_W3 & 0x3f) << 6) | (TL_BARO_W4 & 0x3f);
  c3 = TL_BARO_W4 >> 6;
  c4 = TL_BARO_W3 >> 6;
  uint16_t c5 = ((TL_BARO_W1 & 0x1) << 10) | (TL_BARO_W2 >> 6);
  c6 = TL_BARO_W2 & 0x3f;
  
  ut1 = (c5 << 3) + 20224;


}

void tl_baro_send_req(void) {
  tl_baro_cur_data++; 
  if (tl_baro_cur_data == TL_BARO_NB_DATA) tl_baro_cur_data = TL_BARO_PRESSURE;
  TlBaroSelect();
  //  SpiEnable(); 
  //  SpiEnableRti();
  SpiClrCPHA();
  SSPDR = CMD_MEASUREMENT;
  SSPDR = tl_baro_cmd[tl_baro_cur_data];

}

void tl_baro_read(void) {
  TlBaroSelect();
  SpiEnable();
  SpiEnableRti();
  SpiSetCPHA();
  SSPDR = 0;
  SSPDR = 0;

}

void tl_baro_compute(void) {
  /* Compute pressure and temp (page 10) */
  int16_t dT = tl_baro_d[1] - ut1;
  tl_baro_temp = 200 + (dT*(c6+50)) / (1 << 10);
  int16_t off = c2*4 + ((c4-512)*dT)/(1 << 12);
  uint32_t sens = c1 + (c3*dT)/(1<<10) + 24576;
  uint16_t x = (sens*(tl_baro_d[0]-7168))/(1<<14) - off;
  tl_baro_pressure = ((x*100)>>5) + 250*100;
}
