
#include "std.h"
#include "sys_time.h"
#include "init_hw.h"
#include "interrupt_hw.h"

#include "uart.h"
#include "print.h"

#include "adc.h"
#include "airframe.h"

static void imu_init( void );
static void imu_periodic_task( void );
static void imu_event_task( void);
static void imu_reporting_task ( void );
static void imu_init_spi1( void );
static void imu_read_max1167( void );
static void SPI1_ISR(void) __attribute__((naked));

struct adc_buf buf_ax;
struct adc_buf buf_ay;
struct adc_buf buf_az;
struct adc_buf buf_bat;

volatile uint8_t gotcha = 0;
volatile uint16_t rz = 0;

int main( void ) {
  imu_init();
  while (1) {
    if (sys_time_periodic()) {
      imu_periodic_task();
    }
    imu_event_task();
  }
  return 0;
}

void imu_init( void ) {
  hw_init();
  sys_time_init();
  uart0_init_tx();
  adc_init();
  adc_buf_channel(ADC_CHANNEL_AX, &buf_ax, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_CHANNEL_AY, &buf_ay, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_CHANNEL_AZ, &buf_az, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_CHANNEL_BAT, &buf_bat, DEFAULT_AV_NB_SAMPLE);
  imu_init_spi1();
  int_enable();
}

void imu_event_task( void ) {
  if (gotcha) {
    gotcha = 0;
    Uart0PrintString("GOTCHA ");
    Uart0PrintHex16(rz);
    uart0_transmit('\n');
  }
}

void imu_periodic_task( void ) {
  static uint8_t foo = 0;
  foo++;
  if (!(foo % 10)) {
    imu_reporting_task();
    imu_read_max1167();
  }
}

void imu_reporting_task ( void ) {
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

/*
  MAX1167 connected on SPI1 
  select on P1.29
  eoc on P0.16 ( eint0 )
*/

#define MAX1167_SS_PIN 29

/* SSPCR0 settings */
#define SSP_DDS  0x07 << 0  /* data size         : 8 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : data captured on first clock transition */  
#define SSP_CPHA 0x00 << 7  /* clock phase       :  SCK idles low */
#define SSP_SCR  0x0F << 8  /* serial clock rate : divide by 16  */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE  0x00 << 1  /* SSP enable        : disabled                  */
#define SSP_MS   0x00 << 2  /* master slave mode : master                    */
#define SSP_SOD  0x00 << 3  /* slave output disable : don't care when master */

static void imu_init_spi1( void ) {
  /* configure max1167 CS as output */
  SetBit(IO1DIR, MAX1167_SS_PIN);
  /* unselected max1167 */
  SetBit(IO1SET, MAX1167_SS_PIN);

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
 if (bit_is_set(SSPMIS, RTMIS)) {
   uint8_t foo = SSPDR;
   rz = SSPDR << 8;
   rz += SSPDR;
   gotcha = 1;
   /* clear RTI */
   SetBit(SSPIMSC, RTIM);
   /* disable RTI */
   ClearBit(SSPIMSC, RTIM);
   /* disable SPI */
   ClearBit(SSPCR1, SSE);
   /* unselected max1167 */
   SetBit(IO1SET, MAX1167_SS_PIN);
 }

 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}

static void imu_read_max1167( void ) {
  /* select max1167 */ 
  SetBit(IO1CLR, MAX1167_SS_PIN);
  /* enable SPI */
  SetBit(SSPCR1, SSE);
  /* writecontol byte + 2 dummy bytes for reading */
  uint8_t control_byte = 2<<5;
  SSPDR = control_byte;
  SSPDR = 0;
  SSPDR = 0;
  /* enable timeout interrupt */
  SetBit(SSPIMSC, RTIM);
}
