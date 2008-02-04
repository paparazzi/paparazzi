#include "ADS8344.h"
#include "LPC21xx.h"
#include "armVIC.h"
#include CONFIG

#include "spi_hw.h"

#define ADS8344_SS_IODIR IO0DIR
#define ADS8344_SS_IOSET IO0SET
#define ADS8344_SS_IOCLR IO0CLR
#define ADS8344_SS_PIN   20 

#define ADS8344Select()   SetBit(ADS8344_SS_IOCLR,ADS8344_SS_PIN)
#define ADS8344Unselect() SetBit(ADS8344_SS_IOSET,ADS8344_SS_PIN)

bool_t ADS8344_available;
uint16_t ADS8344_values[NB_CHANNELS];

#define POWER_MODE (1 << 1 | 1)
#define SGL_DIF 1 // Single ended

/* SSPCR0 settings */
#define SSP_DSS  0x07 << 0  /* data size            : 8 bits   */
#define SSP_FRF  0x00 << 4  /* frame format         : SPI      */
#define SSP_CPOL 0x00 << 6  /* clock polarity       : idle low */  
#define SSP_CPHA 0x01 << 7  /* clock phase          : 1        */
#define SSP_SCR  0x0E << 8  /* serial clock rate    : 1MHz     */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode        : disabled */
#define SSP_SSE  0x00 << 1  /* SSP enable           : disabled */
#define SSP_MS   0x00 << 2  /* master slave mode    : master   */
#define SSP_SOD  0x00 << 3  /* slave output disable : disabled */


static void SPI1_ISR(void) __attribute__((naked));
static uint8_t channel;

void ADS8344_init( void ) {
  channel = 0;
  ADS8344_available = FALSE;

  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= 2 << 2 | 2 << 4 | 2 << 6;
  
  /* setup SSP */
  SSPCR0 = SSP_DSS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = 20; /* -> 50kHz */
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);     // SPI1 interrupt enabled
  VICVectCntl7 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr7 = (uint32_t)SPI1_ISR;    // address of the ISR 

  /* setup slave select */
  /* configure SS pin */
  SetBit( ADS8344_SS_IODIR,  ADS8344_SS_PIN);  /* pin is output  */
  ADS8344Unselect();                           /* pin low        */
}

static inline void read_values( void ) {
  uint8_t foo __attribute__ ((unused)) = SSPDR;
  uint8_t msb = SSPDR;
  uint8_t lsb = SSPDR;
  ADS8344_values[channel] = msb << 8 | lsb;
}

static inline void send_request( void ) {
  uint8_t control = 1 << 7 | channel << 4 | SGL_DIF << 2 | POWER_MODE;

  SSPDR = control;
  SSPDR = 0;
  SSPDR = 0;
}

void ADS8344_start( void ) {
  ADS8344Select();
  SpiClearRti();
  SpiEnableRti();
  SpiEnable();
  send_request();
}

void SPI1_ISR(void) {
 ISR_ENTRY();

 read_values();
 channel++;
 if (channel > 7) {
   channel = 0;
   ADS8344_available = TRUE;
 }
 send_request();
 SpiClearRti();

 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}
