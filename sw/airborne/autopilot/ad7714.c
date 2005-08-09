#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>

#include "std.h"
#include "ad7714.h"
#include "spi.h"

/* unipolar, 16 bits, current boost, filter 4000 */
//#define AD_HIGH_FILTER_CFG  0xAF
//#define AD_LOW_FILTER_CFG    0xA0

/*  filter 2000 */
#define AD_HIGH_FILTER_CFG  0xA7
#define AD_LOW_FILTER_CFG   0xD0

/*  filter 19 */
//const uint8_t AD_HIGH_FILTER_CFG = 0xA0;
//const uint8_t AD_LOW_FILTER_CFG  = 0x13;

struct AdMsg {
  uint8_t len;
  const uint8_t* data;
};

#define AD7714_SETUP_LEN 6
const uint8_t ad7714_setup_data[AD7714_SETUP_LEN] = {
  AD_F_HIGH_REG + AD_FD0, /* select high filter register */
  AD_HIGH_FILTER_CFG,
  AD_F_LOW_REG + AD_FD0,  /* select low filter register */
  AD_LOW_FILTER_CFG,
  AD_MODE_REG + AD_FD0,   /* select com register       */
  AD_SELFCAL_MOD + AD_GAIN_128,
};
const struct AdMsg ad7714_setup = {AD7714_SETUP_LEN, ad7714_setup_data};

#define AD7714_READ_LEN 3
const uint8_t ad7714_read_data[] = {
  AD_DATA_REG + AD_WR + AD_FD0, /* transmit read request */
  0x00,  /* transmit a dumb value just to get the result */
  0xAA   /* transmit a dumb value just to get the result */
};
const struct AdMsg ad7714_read = {AD7714_READ_LEN, ad7714_read_data};

#define AD7714_SELECT_CHANNEL_LEN 2
const uint8_t ad7714_select_channel_data[AD7714_SELECT_CHANNEL_LEN] = {
  AD_MODE_REG + AD_FD0,
  //  AD_BG_CAL_MOD + AD_GAIN_128
  AD_NOR_MOD + AD_GAIN_128
};
const struct AdMsg ad7714_select_channel = {AD7714_SELECT_CHANNEL_LEN, ad7714_select_channel_data};


static struct AdMsg* msg;
static uint8_t idx;
uint16_t ad7714_sample;
uint8_t  ad7714_sample_read;


void ad7714_start_transmitting (const struct AdMsg *amsg) {
  /* Enable SPI, Master, MSB first, clock idle high, sample on trailing edge, clock rate fck/128 */
  SPI_START(_BV(SPE)| _BV(MSTR) | _BV(SPR1) | _BV(CPOL) | _BV(CPHA)| _BV(SPR0)); //| _BV(SPR0) 
  SPI_SELECT_SLAVE1();
  msg = (struct AdMsg*)amsg;
  SPDR = msg->data[0];
  idx = 0;
}

void ad7714_on_spi_it( void ) {
  uint8_t spi_read = SPDR;
  if (msg == &ad7714_read) {
    if (idx==1)
      ad7714_sample = spi_read<<8;
    else if (idx==2) {
      ad7714_sample += spi_read;
      ad7714_sample_read = TRUE;
    }
  }
  idx++;
  if (idx < msg->len) {
    SPI_SEND(msg->data[idx]);
  }
  else {
    SPI_UNSELECT_SLAVE1();
    SPI_STOP();
  }
}

uint8_t ad7714_status = 0;
void ad7714_on_it( void ) {
  if (ad7714_status == 0)
    ad7714_start_transmitting(&ad7714_setup);
  else if (ad7714_status == 1)
    ad7714_start_transmitting(&ad7714_select_channel);
  else
    ad7714_start_transmitting(&ad7714_read);
  ad7714_status++;
}

void ad7714_init( void ) {
  /* setupt interrupt on falling edge */
  cbi(EICRB, ISC60);
  sbi(EICRB, ISC61);
  /* clear interrupt flag */
  //    if (bit_is_set(EIFR, INTF6))
  //  EIFR != _BV(INTF6);
  Ad7714_Enable_It();
}


SIGNAL(SIG_INTERRUPT6) {
  ad7714_on_it();
}
