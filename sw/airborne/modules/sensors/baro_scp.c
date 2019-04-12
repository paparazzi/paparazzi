#include "std.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu.h"
#include "subsystems/abi.h"

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#include "mcu_periph/spi.h"

#include "modules/sensors/baro_scp.h"

#ifndef SENSOR_SYNC_SEND
#warning set SENSOR_SYNC_SEND to use baro_scp
#endif


#define STA_UNINIT       0
#define STA_INITIALISING 1
#define STA_IDLE         2

uint8_t  baro_scp_status;
uint32_t baro_scp_pressure;
uint16_t baro_scp_temperature;
bool baro_scp_available;

static void baro_scp_start_high_res_measurement(void);
static void baro_scp_read(void);
static void EXTINT_ISR(void) __attribute__((naked));
static void SPI1_ISR(void) __attribute__((naked));

void baro_scp_periodic(void)
{
  if (baro_scp_status == STA_UNINIT && sys_time.nb_sec > 1) {
    baro_scp_start_high_res_measurement();
    baro_scp_status = STA_INITIALISING;
  }
}

/* ssp input clock 468.75kHz, clock that divided by SCR+1 */
#define SSP_CLOCK 468750

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

#define SS_PIN   20
#define SS_IODIR IO0DIR
#define SS_IOSET IO0SET
#define SS_IOCLR IO0CLR

#define ScpSelect()   SetBit(SS_IOCLR,SS_PIN)
#define ScpUnselect() SetBit(SS_IOSET,SS_PIN)

#warning "This driver should be updated to use the new SPI peripheral"

#ifndef SPI1_VIC_SLOT
#define SPI1_VIC_SLOT 7
#endif

void baro_scp_init(void)
{
  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= 2 << 2 | 2 << 4 | 2 << 6;

  /* setup SSP */
  SSPCR0 = SSP_DDS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  /* set prescaler for SSP clock */
  SSPCPSR = PCLK / SSP_CLOCK;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);     // SPI1 interrupt enabled
  _VIC_CNTL(SPI1_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
  _VIC_CNTL(SPI1_VIC_SLOT) = (uint32_t)SPI1_ISR;    /* address of the ISR */

  /* configure SS pin */
  SetBit(SS_IODIR, SS_PIN); /* pin is output  */
  ScpUnselect();            /* pin idles high */

  /* configure DRDY pin */
  /* connected pin to EXINT */
  SPI1_DRDY_PINSEL |= SPI1_DRDY_PINSEL_VAL << SPI1_DRDY_PINSEL_BIT;
  SetBit(EXTMODE, SPI1_DRDY_EINT); /* EINT is edge trigered */
  SetBit(EXTPOLAR, SPI1_DRDY_EINT); /* EINT is trigered on rising edge */
  SetBit(EXTINT, SPI1_DRDY_EINT);  /* clear pending EINT */

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(SPI1_DRDY_VIC_IT);    /* select EINT as IRQ source */
  VICIntEnable = VIC_BIT(SPI1_DRDY_VIC_IT);      /* enable it */
  VICVectCntl11 = VIC_ENABLE | SPI1_DRDY_VIC_IT;
  VICVectAddr11 = (uint32_t)EXTINT_ISR;         // address of the ISR

  baro_scp_status = STA_UNINIT;
}

void SPI1_ISR(void)
{
  ISR_ENTRY();

  if (baro_scp_status == STA_INITIALISING) {
    uint8_t foo1 = SSPDR;
    uint8_t foo2 = SSPDR;
    baro_scp_status = STA_IDLE;
    foo1 = foo2;
  } else if (baro_scp_status == STA_IDLE) {

    uint8_t foo0 = SSPDR;
    baro_scp_temperature = SSPDR << 8;
    baro_scp_temperature += SSPDR;
    if (baro_scp_temperature & 0x2000) {
      baro_scp_temperature |= 0xC000;
    }
    baro_scp_temperature *= 5;

    uint8_t foo1 = SSPDR;
    uint32_t datard8 = SSPDR << 16;
    uint8_t foo2 = SSPDR;
    baro_scp_pressure = SSPDR << 8;
    baro_scp_pressure += SSPDR;
    baro_scp_pressure += datard8;
    baro_scp_pressure *= 25;
    baro_scp_available = true;
    foo1 = foo2;
    foo0 = foo2;
  }

  ScpUnselect();
  SpiClearRti();
  SpiDisable();

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

void EXTINT_ISR(void)
{
  ISR_ENTRY();
  baro_scp_read();

  SetBit(EXTINT, SPI1_DRDY_EINT); /* clear EINT2 */
  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

/* write 0x0A to 0x03 */
static void baro_scp_start_high_res_measurement(void)
{
  uint8_t cmd  = 0x03 << 2 | 0x02;
  uint8_t data = 0x0A;
  ScpSelect();
  SSPDR = cmd;
  SSPDR = data;
  SpiEnableRti();
  SpiEnable();
}

/* read 0x21 (TEMP), 0x1F (MSB) and 0x20 (LSB) */
static void baro_scp_read(void)
{
  uint8_t cmd0 = 0x21 << 2;
  uint8_t cmd1 = 0x1F << 2;
  uint8_t cmd2 = 0x20 << 2;
  ScpSelect();
  SSPDR = cmd0;
  SSPDR = 0;
  SSPDR = 0;
  SSPDR = cmd1;
  SSPDR = 0;
  SSPDR = cmd2;
  SSPDR = 0;
  SSPDR = 0;
  SpiEnable();
}

void baro_scp_event(void)
{
  if (baro_scp_available == TRUE) {
    uint32_t now_ts = get_sys_time_usec();
    float pressure = (float)baro_scp_pressure;
    AbiSendMsgBARO_ABS(BARO_SCP_SENDER_ID, now_ts, pressure);
#ifdef SENSOR_SYNC_SEND
    DOWNLINK_SEND_SCP_STATUS(DefaultChannel, DefaultDevice, &baro_scp_pressure, &baro_scp_temperature);
#endif
    baro_scp_available = false;
  }
}
