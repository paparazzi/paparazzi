
#include "std.h"
#include "sys_time.h"
#include "init_hw.h"
#include "interrupt_hw.h"

#include "uart.h"
#include "print.h"

#include "adc.h"
#include "max1167.h"
#include "micromag.h"

#include "airframe.h"



static inline void imu_init( void );
static inline void imu_periodic_task( void );
static inline void imu_event_task( void);

static void imu_print_accel_and_bat ( void );
static void imu_init_spi1( void );

static void SPI1_ISR(void) __attribute__((naked));

struct adc_buf buf_ax;
struct adc_buf buf_ay;
struct adc_buf buf_az;
struct adc_buf buf_bat;


#define SPI_SLAVE_NONE 0
#define SPI_SLAVE_MAG 1
#define SPI_SLAVE_MAX 2

uint8_t spi_cur_slave = SPI_SLAVE_NONE;

int main( void ) {
  imu_init();
  while (1) {
    if (sys_time_periodic())
      imu_periodic_task();
    imu_event_task();
  }
  return 0;
}

static inline void imu_init( void ) {
  hw_init();
  sys_time_init();
  uart0_init_tx();
  adc_init();
  adc_buf_channel(ADC_CHANNEL_AX, &buf_ax, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_CHANNEL_AY, &buf_ay, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_CHANNEL_AZ, &buf_az, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_CHANNEL_BAT, &buf_bat, DEFAULT_AV_NB_SAMPLE);
  imu_init_spi1();
  max1167_init();
  micromag_init();
  int_enable();
}

static inline void imu_event_task( void ) {

  if (micromag_data_available) {
    micromag_data_available = FALSE;
    spi_cur_slave = SPI_SLAVE_MAX;
    max1167_read();

    Uart0PrintString("MAG ");
    Uart0PrintHex16(micromag_values[0]);
    uart0_transmit(' ');
    Uart0PrintHex16(micromag_values[1]);
    uart0_transmit(' ');
    Uart0PrintHex16(micromag_values[2]);
    uart0_transmit('\n');
  }

  if (max1167_data_available) {
    max1167_data_available = FALSE;
    spi_cur_slave = SPI_SLAVE_NONE;

    Uart0PrintString("GYRO ");
    Uart0PrintHex16(max1167_values[0]);
    uart0_transmit(' ');
    Uart0PrintHex16(max1167_values[1]);
    uart0_transmit(' ');
    Uart0PrintHex16(max1167_values[2]);
    uart0_transmit('\n');
  }
}

static inline void imu_periodic_task( void ) {
  static uint8_t foo = 0;
  foo++;
  if (!(foo % 10)) {
    if (spi_cur_slave == SPI_SLAVE_NONE) {
      spi_cur_slave = SPI_SLAVE_MAG;
      micromag_read();
    }
    else {
      Uart0PrintString("OVERRUN\n");
    }
    imu_print_accel_and_bat();
  }
}

static void imu_print_accel_and_bat ( void ) {
  Uart0PrintString("ACCEL ");
  uint16_t val = buf_ax.sum / DEFAULT_AV_NB_SAMPLE;
  Uart0PrintHex16(val);
  uart0_transmit(' ');
  val = buf_ay.sum / DEFAULT_AV_NB_SAMPLE;
  Uart0PrintHex16(val);
  uart0_transmit(' ');
  val = buf_az.sum / DEFAULT_AV_NB_SAMPLE;
  Uart0PrintHex16(val);
  uart0_transmit('\n');
  
  Uart0PrintString("BAT ");
  val = buf_bat.sum / DEFAULT_AV_NB_SAMPLE;
  Uart0PrintHex16(val);
  uart0_transmit('\n');

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



static void imu_init_spi1( void ) {
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



