
#include "max11040.h"

volatile uint8_t max11040_status;
volatile uint8_t max11040_data;
volatile int32_t max11040_values[MAX11040_BUF_SIZE][MAXM_NB_CHAN] = {{0}};
volatile uint32_t max11040_timestamp[MAX11040_BUF_SIZE] = {0};
volatile uint8_t max11040_count = 0;
volatile uint32_t max11040_buf_in = 0;
volatile uint32_t max11040_buf_out = 0;

static void SSP_ISR(void) __attribute__((naked));


static void SSP_ISR(void) {
  int i;
  ISR_ENTRY();

  switch (max11040_status) {

    case MAX11040_RESET:
    {
      /* read dummy control byte reply */
      uint8_t foo __attribute__ ((unused));
      foo = SSPDR;
      foo = SSPDR;
      /* write configuration register */
      SSP_Send(0x60);    /* wr conf */
      SSP_Send(0x30);      /* adc0: en24bit, xtalen, no faultdis */
      for (i=1; i<MAXM_NB_ADCS; i++) {
        SSP_Send(0x20);    /* adcx: en24bit, no xtalen, no faultdis */
      }
      max11040_status = MAX11040_CONF;
      SSP_ClearRti();
    }
    break;

    case MAX11040_CONF:
    {
      /* read dummy control byte reply */
      uint8_t foo __attribute__ ((unused));
      foo = SSPDR;
      for (i=0; i<MAXM_NB_ADCS; i++) {
        foo = SSPDR;
      }
      /* write sampling instant register */
      SSP_Send(0x40);    /* wr instant */
      for (i=0; i<MAXM_NB_ADCS; i++) {
        SSP_Send(0);       /* adcx: no delay */
        SSP_Send(0);
        SSP_Send(0);
        SSP_Send(0);
      }
      max11040_status = MAX11040_INSTANT;
      SSP_ClearRti();
    }
    break;

    case MAX11040_INSTANT:
    {
      /* read dummy control byte reply */
      uint8_t foo __attribute__ ((unused));
      foo = SSPDR;
      for (i=0; i<MAXM_NB_ADCS; i++) {
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
      }
      /* write data rate control register */
      SSP_Send(0x50);    /* wr rate */
      SSP_Send(0x26);    /* adc: 250.1 sps */
      SSP_Send(0x00);
      max11040_status = MAX11040_RATE;
      SSP_ClearRti();
    }
    break;

    case MAX11040_RATE:
    {
      uint8_t foo __attribute__ ((unused));
      foo = SSPDR;
      foo = SSPDR;
      foo = SSPDR;
      /* read data register */
      SSP_Send(0xF0);   /* rd data */
      for (i=0; i<MAXM_NB_ADCS; i++) {
        SSP_Send(0x00);   /* adcx: data */
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
      }
      max11040_status = MAX11040_DATA;
      SSP_ClearRti();
    }
    break;

    case MAX11040_DATA:
    {
      uint8_t foo __attribute__ ((unused));
      foo = SSPDR;
      for (i=0; i<MAXM_NB_ADCS; i++) {
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
      }

    /* read data */
      /* read data register */
      SSP_Send(0xF0);   /* rd data */
      for (i=0; i<MAXM_NB_ADCS; i++) {
        SSP_Send(0x00);   /* adc0: data */
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
      }

      SSP_ClearRti();
    }
    break;

    case MAX11040_DATA2:
    {
      uint8_t foo __attribute__ ((unused));

      SSP_ClearRti();
      SSP_ClearRxi();

      if (max11040_count <= MAXM_NB_CHAN+2)
      {
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
      }

      if (max11040_count == 0) foo = SSPDR;

      max11040_values[max11040_buf_in][max11040_count]  = SSPDR << 16;
      max11040_values[max11040_buf_in][max11040_count] |= SSPDR << 8;
      max11040_values[max11040_buf_in][max11040_count] |= SSPDR;
      if (max11040_values[max11040_buf_in][max11040_count] & 0x800000) 
        max11040_values[max11040_buf_in][max11040_count] |= 0xFF000000;

      max11040_count++;

      max11040_values[max11040_buf_in][max11040_count]  = SSPDR << 16;
      max11040_values[max11040_buf_in][max11040_count] |= SSPDR << 8;
      max11040_values[max11040_buf_in][max11040_count] |= SSPDR;
      if (max11040_values[max11040_buf_in][max11040_count] & 0x800000) 
        max11040_values[max11040_buf_in][max11040_count] |= 0xFF000000;

      max11040_count++;

      if (max11040_count == MAXM_NB_CHAN)
      {
        MaxmUnselect();
        max11040_data = MAX11040_DATA_AVAILABLE;
        i = max11040_buf_in+1;
        if (i >= MAX11040_BUF_SIZE) i=0;
        if (i != max11040_buf_out) {
          max11040_buf_in = i;
 	} else {
          max11040_buf_in = i;
	}
      }
    }
    break;

  }

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

void max11040_init_ssp(void) {

  /* setup pins for SSP (SCK, MISO, MOSI, SSEL) */
  PINSEL1 |= SSP_PINSEL1_SCK  | SSP_PINSEL1_MISO | SSP_PINSEL1_MOSI | SSP_PINSEL1_SSEL;
  
  /* setup SSP */
  SSPCR0 = SSPCR0_VAL;;
  SSPCR1 = SSPCR1_VAL;
  SSPCPSR = 0x02;
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT( VIC_SPI1 );  /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT( VIC_SPI1 );    /* enable it            */
  _VIC_CNTL(SSP_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
  _VIC_ADDR(SSP_VIC_SLOT) = (uint32_t)SSP_ISR;      /* address of the ISR   */
}

void max11040_init( void ) {

  int i;
  max11040_hw_init();
  
  max11040_status = MAX11040_RESET;
  max11040_data = MAX11040_RESET;

  /* write configuration register */
  SSP_Send(0x60);       /* wr conf */
  for (i=0; i<MAXM_NB_ADCS; i++) {
    SSP_Send(0x40);     /* adcx: reset */
  }
  SSP_Enable();
  SSP_ClearRti();
  SSP_EnableRti();
}

void max11040_reset() {
  max11040_status = MAX11040_IDLE;
}

