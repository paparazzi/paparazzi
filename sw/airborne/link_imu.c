#include "link_imu.h"

#include "std.h"

struct imu_state link_imu_state;
struct imu_state link_imu_state_foo;


#ifdef IMU  /* IMU LPC code */

#include "imu_v3.h"
#include "ahrs_new.h"

#include "LPC21xx.h"
#include "armVIC.h"



volatile uint8_t spi0_data_available;
volatile uint8_t spi0_idx_buf;
uint8_t* spi0_buffer_output = (uint8_t*)&link_imu_state;
uint8_t* spi0_buffer_input = (uint8_t*)&link_imu_state_foo;

void SPI0_ISR(void) __attribute__((naked));



#define Spi0InitBuf() {						      \
    spi0_idx_buf = 0;						      \
    IO1SET = _BV(SPI0_DRDY);					      \
    /* FIXME : test if last read done */			      \
    S0SPDR = spi0_buffer_output[0];				      \
    IO1CLR = _BV(SPI0_DRDY);					      \
  }

#define Spi0OneByte() {							\
    /* FIXME : do something usefull with the status register reading */ \
    uint8_t foo __attribute__((unused)) = S0SPSR;			\
    spi0_buffer_input[spi0_idx_buf] = S0SPDR;				\
    spi0_idx_buf++;							\
    if (spi0_idx_buf < sizeof(link_imu_state)) {			\
      S0SPDR = spi0_buffer_output[spi0_idx_buf];			\
    }									\
    else {								\
      spi0_data_available = TRUE;					\
      IO1SET = _BV(SPI0_DRDY);						\
    }									\
  } 

/*
  wiring on IMU_V3
 P0_4   SCK0
 P0_5   MISO0
 P0_6   MOSI0
 P0_7   SSEL0
 P1_24  DRDY
*/

#define SPI0_DRDY 24

/*  */
#define S0SPCR_SPIE  7  /* SPI enable */

void link_imu_init ( void ) {
  /* init SPI0 */
  /* setup pins for sck, miso, mosi, SSEL */
  PINSEL0 |= 1<<8 | 1<<10| 1<<12 | 1<< 14;
  /* setup P1_16 to P1_25 as GPIO */
  PINSEL2 &= ~(1<<3);
  /* P1_24 is output */
  SetBit(IO1DIR, SPI0_DRDY);
  /* DRDY idles high */
  SetBit(IO1SET, SPI0_DRDY);
  /* configure SPI : 8 bits CPOL=0 CPHA=0 MSB_first slave */
  S0SPCR = 0;
  /* setup SPI clock rate */
  S0SPCCR = 0x20;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI0);   // SPI0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI0);     // SPI0 interrupt enabled
  VICVectCntl10 = VIC_ENABLE | VIC_SPI0;
  VICVectAddr10 = (uint32_t)SPI0_ISR;    // address of the ISR

  /* clear pending interrupt */
  SetBit(S0SPINT, SPI0IF);
  /* enable SPI interrupt */
  SetBit(S0SPCR, S0SPCR_SPIE);

}

void link_imu_send ( void ) {
  if (!isnan(ahrs_phi)   &&
      !isnan(ahrs_theta) &&
      !isnan(ahrs_psi)) {
    link_imu_state.rates[AXIS_X] = ahrs_p * RATE_PI_S/M_PI;
    link_imu_state.rates[AXIS_Y] = ahrs_q * RATE_PI_S/M_PI;
    link_imu_state.rates[AXIS_Z] = ahrs_r * RATE_PI_S/M_PI;
    link_imu_state.eulers[AXIS_X] = ahrs_phi  * ANGLE_PI/M_PI;
    link_imu_state.eulers[AXIS_Y] = ahrs_theta* ANGLE_PI/M_PI;
    link_imu_state.eulers[AXIS_Z] = ahrs_psi  * ANGLE_PI/M_PI;
    link_imu_state.cos_theta = cos(link_imu_state.eulers[AXIS_Z]);
    link_imu_state.sin_theta = sin(link_imu_state.eulers[AXIS_Z]);
    link_imu_state.status = IMU_RUNNING;
  }
  else {
    link_imu_state.rates[AXIS_X] = imu_gyro[AXIS_X]*RATE_PI_S/M_PI;
    link_imu_state.rates[AXIS_Y] = imu_gyro[AXIS_Y]*RATE_PI_S/M_PI;
    link_imu_state.rates[AXIS_Z] = imu_gyro[AXIS_Z]*RATE_PI_S/M_PI;
    link_imu_state.eulers[AXIS_X] = 0;
    link_imu_state.eulers[AXIS_Y] = 0;
    link_imu_state.eulers[AXIS_Z] = 0;
    link_imu_state.cos_theta = 1.;
    link_imu_state.sin_theta = 0.;
    link_imu_state.status = IMU_CRASHED;
  }
  Spi0InitBuf();
}


void SPI0_ISR(void) {
  ISR_ENTRY();
  Spi0OneByte();
  /* clear the interrupt */
  S0SPINT = _BV(SPI0IF); 
  /* clear this interrupt from the VIC */
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}


#endif /* IMU */





#ifdef CONTROLLER  /* lpc controller board */

#include "LPC21xx.h"
#include "interrupt_hw.h" 


//#include CONFIG
//#include "spi.h"
//#include "fbw_downlink.h"

/* DRDY connected pin to P0.16 EINT0 */ 
#define IMU_DRDY_PINSEL PINSEL1
#define IMU_DRDY_PINSEL_VAL 0x01
#define IMU_DRDY_PINSEL_BIT 0
#define IMU_DRDY_EINT 0

static void EXTINT0_ISR(void) __attribute__((naked));

void link_imu_init ( void ) {
  
  /* configure DRDY pin */
  IMU_DRDY_PINSEL |= IMU_DRDY_PINSEL_VAL << IMU_DRDY_PINSEL_BIT;
  SetBit(EXTMODE, IMU_DRDY_EINT); /* EINT is edge trigered */
  ClearBit(EXTPOLAR,IMU_DRDY_EINT); /* EINT is trigered on falling edge */
  SetBit(EXTINT,IMU_DRDY_EINT);   /* clear pending EINT */
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT( VIC_EINT0 );  /* select EINT0 as IRQ source */
  VICIntEnable = VIC_BIT( VIC_EINT0 );    /* enable it */
  VICVectCntl9 = VIC_ENABLE | VIC_EINT0;
  VICVectAddr9 = (uint32_t)EXTINT0_ISR;   // address of the ISR 

  //  spi_buffer_input = (uint8_t*)&link_imu_state;
  //  spi_buffer_output = (uint8_t*)&link_imu_state_foo;
  //  spi_buffer_length = sizeof(link_imu_state);

  /** Falling edge */
  //  SetBit(EICRB, ISC61); 

  /** Clr pending interrupt */
  //  SetBit(EIFR, INTF6); 

  /** Enable DTRDY interrupt */
  //  SetBit(EIMSK, INT6); 

}

static inline void link_imu_read( void ) {
  //  SpiSelectSlave0();
  //  SpiStart();
}

//SIGNAL( SIG_INTERRUPT6 ) {
  //  link_imu_read();
//}

void link_imu_event_task( void ) {
/*   static uint8_t foo; */
/*   foo++; */
/*   if (foo > 10) foo = 0; */
/*   if (!foo) { */
/*     DOWNLINK_SEND_IMU_SENSORS(3, link_imu_state.rates); */
/*   } */
}

void EXTINT0_ISR(void) {
  ISR_ENTRY();
  /* read dummy control byte reply */
  //  uint8_t foo __attribute__ ((unused)) = SSPDR;
  /* trigger 2 bytes read */
  //  SSPDR = 0;
  //  SSPDR = 0;
  /* enable timeout interrupt */
  //  SpiEnableRti();
  /* clear EINT2 */
  SetBit(EXTINT,IMU_DRDY_EINT); /* clear EINT0 */
  
  VICVectAddr = 0x00000000;    /* clear this interrupt from the VIC */
  ISR_EXIT();
}

#endif /* CONTROLLER */
