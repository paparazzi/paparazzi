#include "link_imu.h"

#include "std.h"

struct imu_state link_imu_state;
struct imu_state link_imu_state_foo;

uint16_t link_imu_crc;

#define LINK_IMU_CRC_INIT 0x0
#define LINK_IMU_PAYLOAD_LENGTH (sizeof(struct imu_state) - 2)
#define LinkImuCrcLow(x) ((x)&0xff)
#define LinkImuCrcHigh(x) ((x)>>8)
#define LinkImuComputeCRC() {						\
    uint8_t i;								\
    link_imu_crc = LINK_IMU_CRC_INIT;					\
    for(i = 0; i < LINK_IMU_PAYLOAD_LENGTH; i++) {			\
      uint8_t _byte = ((uint8_t*)&link_imu_state)[i];			\
      uint8_t a = ((uint8_t)LinkImuCrcHigh(link_imu_crc)) + _byte;	\
      uint8_t b = ((uint8_t)LinkImuCrcLow(link_imu_crc)) + a;		\
      link_imu_crc = b | a << 8;					\
    }									\
  }


#ifdef IMU  /* IMU LPC code */

#include "imu_v3.h"
#include "multitilt.h"

#include "LPC21xx.h"
#include "armVIC.h"

volatile uint8_t spi0_data_available;
volatile uint8_t spi0_idx_buf;
uint8_t* spi0_buffer_output = (uint8_t*)&link_imu_state;
uint8_t* spi0_buffer_input = (uint8_t*)&link_imu_state_foo;

#define LinkImuSetUnavailable() { IO1SET = _BV(SPI0_DRDY); }
#define LinkImuSetAvailable()   { IO1CLR = _BV(SPI0_DRDY); }

void SPI0_ISR(void) __attribute__((naked));

#define Spi0InitBuf() {						      \
    spi0_idx_buf = 0;						      \
    S0SPDR = spi0_buffer_output[0];				      \
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
      LinkImuSetUnavailable();						\
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
  /* P1_24 (DRDY) is output */
  SetBit(IO1DIR, SPI0_DRDY);
  /* DRDY idles high */
  LinkImuSetUnavailable();
  /* configure SPI : 8 bits CPOL=0 CPHA=0 MSB_first slave */
  S0SPCR = _BV(3);
  /* setup SPI clock rate */
  S0SPCCR = 0x20;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI0);   // SPI0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI0);     // SPI0 interrupt enabled
  VICVectCntl1 = VIC_ENABLE | VIC_SPI0;
  VICVectAddr1 = (uint32_t)SPI0_ISR;    // address of the ISR

  /* clear pending interrupt */
  SetBit(S0SPINT, SPI0IF);
  /* enable SPI interrupt */
  SetBit(S0SPCR, S0SPCR_SPIE);

}

void link_imu_send ( void ) {
  LinkImuSetUnavailable();
  
  link_imu_state.r_rates[AXIS_P] = imu_vs_gyro_unbiased[AXIS_P] * RATE_PI_S/M_PI;
  link_imu_state.r_rates[AXIS_Q] = imu_vs_gyro_unbiased[AXIS_Q] * RATE_PI_S/M_PI;
  link_imu_state.r_rates[AXIS_R] = imu_vs_gyro_unbiased[AXIS_R] * RATE_PI_S/M_PI;
  link_imu_state.f_rates[AXIS_P] = mtt_p * RATE_PI_S/M_PI;
  link_imu_state.f_rates[AXIS_Q] = mtt_q * RATE_PI_S/M_PI;
  link_imu_state.f_rates[AXIS_R] = mtt_r * RATE_PI_S/M_PI;
  link_imu_state.f_eulers[AXIS_X] = mtt_phi * ANGLE_PI/M_PI;
  link_imu_state.f_eulers[AXIS_Y] = mtt_theta* ANGLE_PI/M_PI;
  link_imu_state.f_eulers[AXIS_Z] = mtt_psi  * ANGLE_PI/M_PI;
  link_imu_state.status = IMU_RUNNING;

  LinkImuComputeCRC();
  link_imu_state.crc = link_imu_crc;

  Spi0InitBuf();
  LinkImuSetAvailable();
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
/* IMU connected to SSP ( aka SPI1 ) */

#include "LPC21xx.h"
#include "interrupt_hw.h" 
#include "spi.h"

#include "booz_estimator.h"

uint32_t link_imu_nb_err;
uint8_t  link_imu_status;
uint32_t link_imu_timeout;
#define LINK_IMU_TIMEOUT 100

/* DRDY connected pin to P0.16 EINT0 */ 
#define LINK_IMU_DRDY_PINSEL PINSEL1
#define LINK_IMU_DRDY_PINSEL_VAL 0x01
#define LINK_IMU_DRDY_PINSEL_BIT 0
#define LINK_IMU_DRDY_EINT 0

static void EXTINT0_ISR(void) __attribute__((naked));


void link_imu_init ( void ) {

  /* configure DRDY pin */
  LINK_IMU_DRDY_PINSEL |= LINK_IMU_DRDY_PINSEL_VAL << LINK_IMU_DRDY_PINSEL_BIT;
  SetBit(EXTMODE, LINK_IMU_DRDY_EINT);   /* EINT is edge trigered */
  ClearBit(EXTPOLAR,LINK_IMU_DRDY_EINT); /* EINT is trigered on falling edge */
  SetBit(EXTINT,LINK_IMU_DRDY_EINT);     /* clear pending EINT */
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT( VIC_EINT0 );  /* select EINT0 as IRQ source */
  VICIntEnable = VIC_BIT( VIC_EINT0 );    /* enable it */
  VICVectCntl9 = VIC_ENABLE | VIC_EINT0;
  VICVectAddr9 = (uint32_t)EXTINT0_ISR;   // address of the ISR 

  spi_buffer_input = (uint8_t*)&link_imu_state;
  spi_buffer_output = (uint8_t*)&link_imu_state_foo;
  spi_buffer_length = sizeof(link_imu_state);

  link_imu_nb_err = 0;
  link_imu_status = IMU_NO_LINK;
  link_imu_timeout = LINK_IMU_TIMEOUT;

}

void link_imu_event_task( void ) {
  LinkImuComputeCRC();
  if (link_imu_crc == link_imu_state.crc) {
    link_imu_timeout = 0;
    link_imu_status = link_imu_state.status;
    booz_estimator_uf_p = link_imu_state.r_rates[AXIS_P] * M_PI/RATE_PI_S;
    booz_estimator_uf_q = link_imu_state.r_rates[AXIS_Q] * M_PI/RATE_PI_S;
    booz_estimator_uf_r = link_imu_state.r_rates[AXIS_R] * M_PI/RATE_PI_S;
    booz_estimator_p = link_imu_state.f_rates[AXIS_P] * M_PI/RATE_PI_S;
    booz_estimator_q = link_imu_state.f_rates[AXIS_Q] * M_PI/RATE_PI_S;
    booz_estimator_r = link_imu_state.f_rates[AXIS_R] * M_PI/RATE_PI_S;
    booz_estimator_phi   = link_imu_state.f_eulers[AXIS_X] * M_PI/ANGLE_PI;
    booz_estimator_theta = link_imu_state.f_eulers[AXIS_Y] * M_PI/ANGLE_PI;
    booz_estimator_psi   = link_imu_state.f_eulers[AXIS_Z] * M_PI/ANGLE_PI;
  }
  else {
    link_imu_nb_err++;
  }
}

extern void link_imu_periodic_task( void ) {
  if (link_imu_timeout < LINK_IMU_TIMEOUT)
    link_imu_timeout++;
  else
    link_imu_status = IMU_NO_LINK;
}


void EXTINT0_ISR(void) {
  ISR_ENTRY();

  SpiSelectSlave0();
  SpiStart();

  /* clear EINT2 */
  SetBit(EXTINT,LINK_IMU_DRDY_EINT); /* clear EINT0 */
  
  VICVectAddr = 0x00000000;    /* clear this interrupt from the VIC */
  ISR_EXIT();
}

#endif /* CONTROLLER */
