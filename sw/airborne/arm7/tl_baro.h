#ifndef TL_BARO_H
#define TL_BARO_H

#include "std.h"
#include "LPC21xx.h"
#include "spi_hw.h"
#include "sys_time.h"

#define TL_BARO_PRESSURE 0
#define TL_BARO_TEMP     1
#define TL_BARO_NB_DATA  2

extern uint8_t tl_baro_cur_data;
extern uint16_t tl_baro_d[];

extern uint32_t tl_baro_pressure;
extern uint16_t tl_baro_temp;


/* P0.28 */
#define TL_BARO_SS_IODIR IO0DIR
#define TL_BARO_SS_PIN   28 
#define TL_BARO_SS_IOCLR IO0CLR
#define TL_BARO_SS_IOSET IO0SET

#define TlBaroUnselect()   SetBit(TL_BARO_SS_IOCLR,TL_BARO_SS_PIN)
#define TlBaroSelect() SetBit(TL_BARO_SS_IOSET,TL_BARO_SS_PIN)

#define TlBaroOnSpiIntReading() {	    \
    tl_baro_d[tl_baro_cur_data] = SSPDR<<8; \
    tl_baro_d[tl_baro_cur_data] |= SSPDR;   \
    TlBaroUnselect();			    \
    SpiDisable();			    \
    SpiDisableRti();			    \
  }

#define TlBaroOnSpiIntSending() {  \
    uint8_t foo  __attribute__ ((unused)) = SSPDR;			\
    uint8_t foo1 __attribute__ ((unused)) = SSPDR;			\
    TlBaroUnselect();							\
    SpiDisable();							\
    SpiDisableRti();							\
  }


extern void tl_baro_init(void);
extern void tl_baro_send_req(void);
extern void tl_baro_read(void);
extern void tl_baro_compute(void);
#endif /* TL_BARO_H */
