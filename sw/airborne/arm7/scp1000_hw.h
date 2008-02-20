#ifndef SCP1000_HW_H
#define SCP1000_HW_H


#include "LPC21xx.h"
#include "interrupt_hw.h"  
#include "spi_hw.h"

#include "booz_debug.h"

/* FIXME : polling mode cause I'm loosing EOC interrupt */
#define SCP1000_NO_EINT

#define SCP1000_ERR_STATUS 0


/* SS on P0.2 */
#define SCP_SS_IODIR IO0DIR
#define SCP_SS_IOSET IO0SET
#define SCP_SS_IOCLR IO0CLR
#define SCP_SS_IOPIN IO0PIN
//#define SCP_SS_PIN   2
#define SCP_SS_PIN   14

/* DRDY on P0.3 ( EINT1 ) */
#define SCP_DRDY_IOPIN  IO0PIN
#define SCP_DRDY_PIN    3
#define SCP_DRDY_PINSEL PINSEL0
#define SCP_DRDY_PINSEL_BIT   6
#define SCP_DRDY_PINSEL_VAL   3
#define SCP_DRDY_EINT         1
#define SCP_DRDY_VIC_IT       VIC_EINT1

#define SCP_MEAS_HIGH_SPEED 0x09
#define SCP_MEAS_HIGH_RES   0x0A
#define SCP_MEAS_MODE SCP_MEAS_HIGH_SPEED

#define Scp1000Unselect() SetBit(SCP_SS_IOSET,SCP_SS_PIN)
#define Scp1000Select()   SetBit(SCP_SS_IOCLR,SCP_SS_PIN)

#define Scp1000DataReady() bit_is_set(SCP_DRDY_IOPIN, SCP_DRDY_PIN)

#define Scp1000SendConfig() {					\
    ASSERT((scp1000_status == SCP1000_STA_STOPPED),		\
	   DEBUG_SCP1000, SCP1000_ERR_STATUS);			\
    const uint8_t cmd  = 0x03<<2|0x02;				\
    const uint8_t data = SCP_MEAS_MODE;				\
    Scp1000Select();						\
    SpiClearRti();						\
    SpiEnableRti();						\
    SSPDR = cmd;						\
    SSPDR = data;						\
    SpiEnable();						\
  }

#define Scp1000Read() {						\
    ASSERT((scp1000_status == SCP1000_STA_GOT_EOC),		\
	   DEBUG_SCP1000, SCP1000_ERR_STATUS);			\
    const uint8_t cmd1 = 0x1F << 2;				\
    const uint8_t cmd2 = 0x20 << 2;				\
    Scp1000Select();						\
    scp1000_status = SCP1000_STA_SENDING_REQUEST;		\
    SpiClearRti();						\
    SpiEnableRti();						\
    SSPDR = cmd1;						\
    SSPDR = 0;							\
    SSPDR = cmd2;						\
    SSPDR = 0;							\
    SSPDR = 0;							\
    SpiEnable();						\
  }

/* FIXME READ high bit last */

#define Scp1000OnSpiIt() {				\
    switch (scp1000_status) {				\
    case SCP1000_STA_STOPPED:				\
      {							\
	uint8_t foo1 __attribute__ ((unused)) = SSPDR;	\
	uint8_t foo2 __attribute__ ((unused)) = SSPDR;	\
	Scp1000Unselect();				\
	SpiClearRti();					\
	SpiDisableRti();				\
	SpiDisable();					\
	scp1000_status = SCP1000_STA_WAIT_EOC;		\
      }							\
      break;						\
    case SCP1000_STA_SENDING_REQUEST:			\
      {							\
	uint8_t foo1 __attribute__ ((unused)) = SSPDR;	\
	uint32_t high_bit = SSPDR<<16;			\
	uint8_t foo2 __attribute__ ((unused)) = SSPDR;	\
	scp1000_pressure = SSPDR<<8;			\
	scp1000_pressure += SSPDR;			\
	scp1000_pressure += high_bit;			\
	Scp1000Unselect();				\
	SpiClearRti();					\
	SpiDisableRti();				\
	SpiDisable();					\
	scp1000_status = SCP1000_STA_DATA_AVAILABLE;	\
      }							\
	break;						\
    }							\
  }


#endif /* SCP1000_HW_H */
