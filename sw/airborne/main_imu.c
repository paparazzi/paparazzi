
#include "std.h"
#include "sys_time.h"
#include "init_hw.h"
#include "interrupt_hw.h"

#include "uart.h"
#include "print.h"

#include "adc.h"
#include "max1167.h"
#include "micromag.h"
#include "imu_v3.h"
//#include "ahrs_new.h"
#include "multitilt.h"
#include "link_imu.h"

#include "messages.h"
#include "downlink.h"

#define SEND_ACCEL 1
#define SEND_MAG   1
#define SEND_GYRO  1
#define SEND_AHRS_STATE 1
//#define SEND_AHRS_COV   1

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void);

static inline void ahrs_task( void );

static void main_init_spi1( void );

static void SPI1_ISR(void) __attribute__((naked));

#define SPI_SLAVE_NONE 0
#define SPI_SLAVE_MAG  1
#define SPI_SLAVE_MAX  2

uint8_t spi_cur_slave = SPI_SLAVE_NONE;


#define AHRS_STEP_UNINIT -1
#define AHRS_STEP_ROLL    0
#define AHRS_STEP_PITCH   1
#define AHRS_STEP_YAW     2
#define AHRS_STEP_NB      3
int8_t ahrs_step = AHRS_STEP_UNINIT;

int main( void ) {
  main_init();
  while (1) {
    if (sys_time_periodic())
      main_periodic_task();
    main_event_task();
  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
  uart1_init_tx();
  adc_init();
  imu_init();
  main_init_spi1();
  max1167_init();
  micromag_init();
  link_imu_init();
  int_enable();
}

static uint32_t t0, t1;

static inline void main_event_task( void ) {

  if (micromag_data_available) {
    micromag_data_available = FALSE;
    spi_cur_slave = SPI_SLAVE_NONE;
    ImuUpdateMag();
#ifdef SEND_MAG
    DOWNLINK_SEND_IMU_MAG(&imu_mag[AXIS_X], &imu_mag[AXIS_Y], &imu_mag[AXIS_Z]);
#endif
    spi_cur_slave = SPI_SLAVE_MAX;
    max1167_read();
  }

  if (max1167_data_available) {
    max1167_data_available = FALSE;
    spi_cur_slave = SPI_SLAVE_NONE;
    ImuUpdateGyros();
#ifdef SEND_GYRO
    DOWNLINK_SEND_IMU_GYRO(&imu_gyro[AXIS_X], &imu_gyro[AXIS_Y], &imu_gyro[AXIS_Z]);
#endif
    ImuUpdateAccels();
#ifdef SEND_ACCEL
    DOWNLINK_SEND_IMU_ACCEL(&imu_accel[AXIS_X], &imu_accel[AXIS_Y], &imu_accel[AXIS_Z]);
#endif
    //    ahrs_task();

    t1=T0TC;
    //    uint32_t dt = t1 - t0;
    //    DOWNLINK_SEND_TIME(&dt);

    link_imu_send();
  }
}

static inline void main_periodic_task( void ) {
  if (spi_cur_slave != SPI_SLAVE_NONE)
    DOWNLINK_SEND_AHRS_OVERRUN();
  t0 = T0TC;

  // spi_cur_slave = SPI_SLAVE_MAX;
  //  max1167_read();

  spi_cur_slave = SPI_SLAVE_MAG;
  micromag_read();

}

#if 0
static inline void ahrs_task( void ) {
  /* discard first 100 measures */
  if (ahrs_step == AHRS_STEP_UNINIT) {
    static uint8_t init_count = 0;
    init_count++;
    if (init_count > 100) {
      ahrs_step = AHRS_STEP_ROLL;
      ahrs_init(imu_mag, imu_accel, imu_gyro);
    }
  }
  else {
    ahrs_predict(imu_gyro);
    switch (ahrs_step) {
    case AHRS_STEP_ROLL: 
      ahrs_roll_update(imu_accel); 
      break;
    case AHRS_STEP_PITCH:
      ahrs_pitch_update(imu_accel); 
      break;
    case AHRS_STEP_YAW:  
      ahrs_yaw_update(imu_mag);
      break;
    }
#ifdef SEND_AHRS_STATE
    DOWNLINK_SEND_AHRS_STATE(&q0, &q1, &q2, &q3, &bias_p, &bias_q, &bias_r);
#endif
#ifdef SEND_AHRS_COV
    DOWNLINK_SEND_AHRS_COV(&P[0][0], &P[1][1]);
#endif
    ahrs_step++;
    if (ahrs_step == AHRS_STEP_NB) ahrs_step = AHRS_STEP_ROLL;
  }
}
#endif

/* SSPCR0 settings */
#define SSP_DDS  0x07 << 0  /* data size         : 8 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : data captured on first clock transition */  
#define SSP_CPHA 0x00 << 7  /* clock phase       : SCK idles low */
#define SSP_SCR  0x0F << 8  /* serial clock rate : divide by 16  */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE  0x00 << 1  /* SSP enable        : disabled                  */
#define SSP_MS   0x00 << 2  /* master slave mode : master                    */
#define SSP_SOD  0x00 << 3  /* slave output disable : don't care when master */

static void main_init_spi1( void ) {
   /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= 2 << 2 | 2 << 4 | 2 << 6;
  
  /* setup SSP */
  SSPCR0 = SSP_DDS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = 0x20;
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);     // SPI1 interrupt enabled
  VICVectCntl7 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr7 = (uint32_t)SPI1_ISR;    // address of the ISR 
}

void SPI1_ISR(void) {
 ISR_ENTRY();

 switch (spi_cur_slave) {
 case SPI_SLAVE_MAG :
   MmOnSpiIt();
   break;
 case SPI_SLAVE_MAX :
   Max1167OnSpiInt();
   break;
 }

 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}



