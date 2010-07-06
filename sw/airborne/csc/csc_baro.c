#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"

#include "uart.h"
#include "messages.h"
#include "downlink.h"
#include "csc_booz2_ins.h"
#include "csc_ap_link.h"

#include "spi_hw.h"

#include "csc_baro.h"

uint8_t  baro_scp_status;
uint32_t baro_scp_pressure;
uint16_t baro_scp_temperature;
float baro_scp_alt;
bool_t baro_scp_available;

static void baro_scp_start_high_res_measurement(void);
static void spi1_isr(void);

void baro_scp_periodic(void) {
  if (cpu_time_sec > 1) {
    if (baro_scp_status == STA_UNINIT) {
      baro_scp_start_high_res_measurement();
      baro_scp_status = STA_INITIALISING;
    } else {
      spi1_isr();
    }
  }
}

#define SS_PIN  30
#define SS_IODIR IO0DIR
#define SS_IOSET IO0SET
#define SS_IOCLR IO0CLR

#define ScpSelect()   SetBit(SS_IOCLR,SS_PIN)
#define ScpUnselect() SetBit(SS_IOSET,SS_PIN)

// DRDY1 aka P0.15
#define DRDY1_PIN 15
#define DRDY1_IODIR IO0DIR
#define DRDY1_IOPIN IO0PIN
#define DRDY1_PINSEL() PINSEL0 = (PINSEL0 & ~(3 << 30)) | (0 << 30)

#define SSP1_SCK_PINSEL() (PINSEL1 = (PINSEL1 & ~(3 << 2)) | (2 << 2))
#define SSP1_MISO_PINSEL() (PINSEL1 = (PINSEL1 & ~(3 << 4)) | (2 << 4))
#define SSP1_MOSI_PINSEL() (PINSEL1 = (PINSEL1 & ~(3 << 6)) | (2 << 6))
#define SSP1_SSEL_PINSEL() (PINSEL1 = (PINSEL1 & ~(3 << 8)) | (2 << 8))

static uint8_t baro_scp_read_reg_8(uint8_t regno)
{
  uint8_t value;
  uint8_t foo0 __attribute__ ((unused));

  ScpSelect();
  sys_time_usleep(20);
  S1SPDR = regno << 2;
  while(~S1SPSR & _BV(7));
  foo0 = S1SPDR;

  S1SPDR = 0;
  while(~S1SPSR & _BV(7));
  value = S1SPDR;
  sys_time_usleep(10);
  ScpUnselect();

  return value;
}

static void baro_scp_write_reg_8(uint8_t regno, uint8_t value)
{
  uint8_t foo0 __attribute__ ((unused));
  uint8_t foo1 __attribute__ ((unused));

  ScpSelect();

  foo0 = S1SPSR;
  foo0 = S1SPDR;

  sys_time_usleep(20);
  S1SPDR = (regno << 2) | (1 << 1);
  while(~S1SPSR & _BV(7));
  foo0 = S1SPDR;

  S1SPDR = value;
  foo1 =  S1SPDR;
  while(~S1SPSR & _BV(7));

  sys_time_usleep(10);
  ScpUnselect();
}

static uint16_t baro_scp_read_reg_16(uint8_t regno)
{
  uint8_t lsb, msb;
  uint8_t foo0 __attribute__ ((unused));

  ScpSelect();
  sys_time_usleep(20);
  S1SPDR = regno << 2;
  while(~S1SPSR & _BV(7));
  foo0 = S1SPDR;

  S1SPDR = 0;
  while(~S1SPSR & _BV(7));
  msb = S1SPDR;

  S1SPDR = 0;
  while(~S1SPSR & _BV(7));
  lsb = S1SPDR;

  sys_time_usleep(10);
  ScpUnselect();


  return (uint16_t) ((msb << 8) | lsb);
}

void baro_scp_init( void )
{
  
  SSP1_SCK_PINSEL();
  SSP1_MISO_PINSEL();
  SSP1_MOSI_PINSEL();
  SSP1_SSEL_PINSEL();
  DRDY1_PINSEL();
  
  S1SPCR = _BV(3) | _BV(4) | _BV(5);
  S1SPCCR = 0x80;

  baro_scp_status = STA_UNINIT;
}

void spi1_isr()
{
  uint32_t msb;
  uint16_t lsb;

  if (bit_is_set(DRDY1_IOPIN, DRDY1_PIN)) {
    sys_time_usleep(10);
    baro_scp_temperature = baro_scp_read_reg_16(0x21);
    msb  = baro_scp_read_reg_8(0x1F);
    lsb  = baro_scp_read_reg_16(0x20);

    //if (baro_scp_temperature & 0x2000) {
     // baro_scp_temperature |= 0xC000;
    //}
    baro_scp_temperature = baro_scp_temperature & (0xFFFF >> 2);
    baro_scp_temperature *= 5;
  
    baro_scp_pressure = lsb;
    baro_scp_pressure |= (msb << 16);
    baro_scp_pressure *= 25;

    //baro_scp_alt = 9000 - baro_scp_pressure * 0.00084366;
    baro_scp_status = STA_VALID;
  }
}

/* write 0x0A to 0x03 */
static void baro_scp_start_high_res_measurement(void) {
  baro_scp_write_reg_8(0x3, 0xA);
}
