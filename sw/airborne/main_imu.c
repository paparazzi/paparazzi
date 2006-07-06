
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
#include "ahrs_new.h"
#include "link_imu.h"

#include "messages.h"
#include "downlink.h"


static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void);

//struct adc_buf buf_ax;
struct adc_buf buf_ay;
//struct adc_buf buf_az;
//struct adc_buf buf_bat;

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
  //  spi0_init();
  link_imu_init();
  int_enable();
}

static uint32_t t0, t1;

static inline void main_event_task( void ) {

  if (micromag_data_available) {
    micromag_data_available = FALSE;
    spi_cur_slave = SPI_SLAVE_NONE;
    ImuUpdateMag();
    //    DOWNLINK_SEND_AHRS_MAG(&imu_mag[AXIS_X], &imu_mag[AXIS_Y], &imu_mag[AXIS_Z]);
    spi_cur_slave = SPI_SLAVE_MAX;
    max1167_read();
  }

  if (max1167_data_available) {
    max1167_data_available = FALSE;
    spi_cur_slave = SPI_SLAVE_NONE;
    ImuUpdateGyros();
    ImuUpdateAccels();
    ahrs_task();

    t1=T0TC;
    uint32_t dt = t1 - t0;
    //    DOWNLINK_SEND_TIME(&dt);

    link_imu_state.rates[AXIS_X] = imu_gyro[AXIS_X];
    link_imu_state.rates[AXIS_Y] = imu_gyro[AXIS_Y];
    link_imu_state.rates[AXIS_Z] = imu_gyro[AXIS_Z];
    link_imu_state.eulers[AXIS_X] = ahrs_euler[AXIS_X];
    link_imu_state.eulers[AXIS_Y] = ahrs_euler[AXIS_Y];
    link_imu_state.eulers[AXIS_Z] = ahrs_euler[AXIS_Z];
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


static inline void ahrs_task( void ) {


  ahrs_pqr[AXIS_X] = imu_gyro[AXIS_X];
  ahrs_pqr[AXIS_Y] = imu_gyro[AXIS_Y];
  ahrs_pqr[AXIS_Z] = imu_gyro[AXIS_Z];

  switch (ahrs_step) {
  case AHRS_STEP_UNINIT : {
    static uint8_t init_count = 0;
    init_count++;
    if (init_count < 100) ahrs_step--;
    else {
      ahrs_euler[AXIS_X] = ahrs_roll_of_accel(imu_accel);
      ahrs_euler[AXIS_Y] = ahrs_pitch_of_accel(imu_accel);
      ahrs_euler[AXIS_Z] = 0.;
      imu_mag[AXIS_X] = 100; imu_mag[AXIS_Y] = 0;  imu_mag[AXIS_Z] = 0; 
      ahrs_init(imu_mag);
    }
  }
    break;
  case AHRS_STEP_ROLL: 
    ahrs_state_update();
    ahrs_roll_update(ahrs_roll_of_accel(imu_accel)); 
    break;
  case AHRS_STEP_PITCH:
    ahrs_state_update();
    ahrs_pitch_update(ahrs_pitch_of_accel(imu_accel)); 
    break;
  case AHRS_STEP_YAW:  
    ahrs_state_update();
    ahrs_compass_update(ahrs_heading_of_mag(imu_mag));
    DOWNLINK_SEND_AHRS(&q0, &q1, &q2, &q3, &bias_p, &bias_q, &bias_r);
    //DOWNLINK_SEND_AHRS2(&ahrs_euler[AXIS_X], &ahrs_euler[AXIS_Y], &ahrs_euler[AXIS_Z]);
    break;
  }
  ahrs_step++;
  if (ahrs_step == AHRS_STEP_NB) ahrs_step = AHRS_STEP_ROLL;
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



