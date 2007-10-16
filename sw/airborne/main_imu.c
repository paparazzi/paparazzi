
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
//#include "multitilt.h"
#include "ahrs_quat_fast_ekf.h"
#include "link_imu.h"

#include "messages.h"
#include "downlink.h"

//#define SEND_ACCEL 1
//#define SEND_MAG   1
#define SEND_GYRO  1
//#define SEND_ACCEL_RAW 1
//#define SEND_MAG_RAW   1
#define SEND_GYRO_RAW  1
//#define SEND_ACCEL_RAW_AVG 1
//#define SEND_MAG_RAW_AVG   1
//#define SEND_GYRO_RAW_AVG  1
//#define SEND_AHRS_STATE 1
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
#ifdef SEND_MAG_RAW
    DOWNLINK_SEND_IMU_MAG_RAW(&imu_mag_raw[AXIS_X], &imu_mag_raw[AXIS_Y], &imu_mag_raw[AXIS_Z]);
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
#ifdef SEND_GYRO_RAW
    DOWNLINK_SEND_IMU_GYRO_RAW(&imu_gyro_raw[AXIS_X], &imu_gyro_raw[AXIS_Y], &imu_gyro_raw[AXIS_Z]);
#endif
    ImuUpdateAccels();
#ifdef SEND_ACCEL
    DOWNLINK_SEND_IMU_ACCEL(&imu_accel[AXIS_X], &imu_accel[AXIS_Y], &imu_accel[AXIS_Z]);
#endif
#ifdef SEND_ACCEL_RAW
    DOWNLINK_SEND_IMU_ACCEL_RAW(&imu_accel_raw[AXIS_X], &imu_accel_raw[AXIS_Y], &imu_accel_raw[AXIS_Z]);
#endif

    ahrs_task();
    
    link_imu_send();
  }
}

static inline void main_periodic_task( void ) {
  if (spi_cur_slave != SPI_SLAVE_NONE)
    DOWNLINK_SEND_AHRS_OVERRUN();
#if 0  
  t0 = T0TC;
  afe_predict( imu_gyro );
  t1 = T0TC;
  uint32_t dif = t1 - t0;
  DOWNLINK_SEND_TIME(&dif);
#endif
  // spi_cur_slave = SPI_SLAVE_MAX;
  //  max1167_read();

  spi_cur_slave = SPI_SLAVE_MAG;
  micromag_read();

}

static inline void ahrs_task( void ) {

  if (ahrs_step == AHRS_STEP_UNINIT) {
    imu_detect_vehicle_still();
#ifdef SEND_GYRO_RAW_AVG
    DOWNLINK_SEND_IMU_GYRO_RAW_AVG(&imu_vs_gyro_raw_avg[AXIS_X], &imu_vs_gyro_raw_avg[AXIS_Y], &imu_vs_gyro_raw_avg[AXIS_Z], \
				   &imu_vs_gyro_raw_var[AXIS_X], &imu_vs_gyro_raw_var[AXIS_Y], &imu_vs_gyro_raw_var[AXIS_Z]);
#endif
#ifdef SEND_ACCEL_RAW_AVG
    DOWNLINK_SEND_IMU_ACCEL_RAW_AVG(&imu_vs_accel_raw_avg[AXIS_X], &imu_vs_accel_raw_avg[AXIS_Y], &imu_vs_accel_raw_avg[AXIS_Z], \
				    &imu_vs_accel_raw_var[AXIS_X], &imu_vs_accel_raw_var[AXIS_Y], &imu_vs_accel_raw_var[AXIS_Z]);
#endif
#ifdef SEND_MAG_RAW_AVG
    DOWNLINK_SEND_IMU_MAG_RAW_AVG(&imu_vs_mag_raw_avg[AXIS_X], &imu_vs_mag_raw_avg[AXIS_Y], &imu_vs_mag_raw_avg[AXIS_Z], \
				  &imu_vs_mag_raw_var[AXIS_X], &imu_vs_mag_raw_var[AXIS_Y], &imu_vs_mag_raw_var[AXIS_Z]);
#endif
    //    if (imu_vehicle_still) {
    //      ahrs_step = AHRS_STEP_ROLL;
    //      afe_init(imu_mag, imu_accel, imu_gyro);
    //}
  }
  else {
    afe_predict(imu_gyro);
    switch (ahrs_step) {
    case AHRS_STEP_ROLL: 
      afe_update_phi(imu_accel); 
      break;
    case AHRS_STEP_PITCH:
      afe_update_theta(imu_accel); 
      break;
    case AHRS_STEP_YAW:  
      afe_update_psi(imu_mag);
      break;
    }
#ifdef SEND_AHRS_STATE
    DOWNLINK_SEND_AHRS_STATE(&afe_q0, &afe_q1, &afe_q2, &afe_q3, &afe_bias_p, &afe_bias_q, &afe_bias_r);
#endif
#ifdef SEND_AHRS_COV
    DOWNLINK_SEND_AHRS_COV(&afe_P[0][0], &afe_P[1][1], &afe_P[2][2], &afe_P[3][3], &afe_P[4][4], &afe_P[5][5], &afe_P[6][6]);
#endif
    ahrs_step++;
    if (ahrs_step == AHRS_STEP_NB) ahrs_step = AHRS_STEP_ROLL;
  }
}

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



