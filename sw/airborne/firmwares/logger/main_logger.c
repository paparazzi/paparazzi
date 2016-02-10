#define DEFAULT_AC_ID 23
#define LOG_OTF 1
/*
 * $Id$
 *
 * Copyright (C) 2009  Martin Mueller
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file main_logger.c
 *  \brief Logger application
 *
 *   This collects telemetry received through a serial port and writes that
 * to a (micro) SD card through the efsl library
 */

  /* XBee-message: ABCDxxxxxxxE
     A XBEE_START (0x7E)
     B LENGTH_MSB (D->D)
     C LENGTH_LSB
     D XBEE_PAYLOAD
       0 XBEE_TX16 (0x01) / XBEE_RX16 (0x81)
       1 FRAME_ID (0)     / SRC_ID_MSB
       2 DEST_ID_MSB      / SRC_ID_LSB
       3 DEST_ID_LSB      / XBEE_RSSI
       4 TX16_OPTIONS (0) / RX16_OPTIONS
       5 PPRZ_DATA
         0 SENDER_ID
         1 MSG_ID
         2 MSG_PAYLOAD
         . DATA (messages.xml)
     E XBEE_CHECKSUM (sum[A->D])

    ID is AC_ID for aircraft, 0x100 for ground station
  */

  /* PPRZ-message: ABCxxxxxxxDE
     A PPRZ_STX (0x99)
     B LENGTH (A->E)
     C PPRZ_DATA
       0 SENDER_ID
       1 MSG_ID
       2 MSG_PAYLOAD
       . DATA (messages.xml)
     D PPRZ_CHECKSUM_A (sum[B->C])
     E PPRZ_CHECKSUM_B (sum[ck_a])
  */

  /* LOG-message: ABCDEFGHxxxxxxxI
     A PPRZ_STX (0x99)
     B LENGTH (H->H)
     C SOURCE (0=uart0, 1=uart1, 2=i2c0, ...)
     D TIMESTAMP_LSB (100 microsec raster)
     E TIMESTAMP
     F TIMESTAMP
     G TIMESTAMP_MSB
     H PPRZ_DATA
       0 SENDER_ID
       1 MSG_ID
       2 MSG_PAYLOAD
       . DATA (messages.xml)
     I CHECKSUM (sum[B->H])
  */

#include "std.h"
#include "mcu.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "usb_msc_hw.h"

#include "efs.h"
#include "ls.h"

#ifdef USE_MAX11040
#include "max11040.h"
#endif

#include "LPC21xx.h"


#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE (!FALSE)
#endif

#ifndef LOG_STOP_KEY_
/* BUTTON that stops logging (PPM_IN = P0.6, BUTTON = P0.7, DTR = P0.13, INT1 = P0.14) */
#define LOG_STOP_KEY 14
#endif

/*
#ifndef POWER_DETECT_PIN
// Pin 0.10
#define POWER_DETECT_PIN 6
#endif

#ifndef CARD_DETECT_PIN
// Pin 1.20
#define CARD_DETECT_PIN 20
#endif
*/

#ifndef LED_GREEN
#define LED_GREEN	3
#endif

#ifndef LED_YELLOW
#define LED_YELLOW	2
#endif

#ifndef LED_RED
#define LED_RED		1
#endif


/* USB Vbus (= P0.23) */
#define VBUS_PIN 23

/** Constants for the API protocol */
#define XBEE_START 0x7e
#define XBEE_TX16_ID 0x01
#define TX16_OPTIONS 0x00
#define NO_FRAME_ID 0
#define XBEE_RFDATA_OFFSET 5
#define XBEE_RX16_ID 0x81

/** Status of the API packet receiver automata */
#define XBEE_UNINIT 0
#define XBEE_GOT_START 1
#define XBEE_GOT_LENGTH_MSB 2
#define XBEE_GOT_LENGTH_LSB 3
#define XBEE_GOT_PAYLOAD 4
#define XBEE_PAYLOAD_LEN 256

/** Receiving pprz messages */
#define STX  0x99
#define UNINIT 0
#define GOT_STX 1
#define GOT_LENGTH 2
#define GOT_PAYLOAD 3
#define GOT_CRC1 4
#define PPRZ_PAYLOAD_LEN 256
#define PPRZ_DATA_OFFSET 2

/** logging messages **/
#define LOG_DATA_OFFSET 7
#define MSG_SIZE 256
/* logging frequency in Hz */
#define LOG_FREQ 10000
/* T0_CLK = PCLK / T0_PCLK_DIV (shall be 15MHz)
   frequency = T0_CLK / LOG_FREQ (10kHz = 100micro seconds) */
#define LOG_DIV ((PCLK / T0_PCLK_DIV) / LOG_FREQ)

#define LOG_SOURCE_UART0    0
#define LOG_SOURCE_UART1    1
#define LOG_SOURCE_I2C0     2
#define LOG_SOURCE_I2C1     3

#define OTF_UNINIT         0x00
#define OTF_WAIT_START     OTF_UNINIT
#define OTF_WAIT_COUNTER   0x01
#define OTF_WAIT_ANGLES    0x02
#define OTF_WAIT_ALTITUDE  0x03
#define OTF_WAIT_CHECKSUM  0x04

#define OTF_START   0x0A
#define OTF_LIMITER ','
#define OTF_END     0x0D

/* workaround for newlib */
void* _sbrk(int);
void* _sbrk(int a) {return 0;}

static inline void main_init( void );
static inline void main_periodic_task( void );
int main_log(void);

void set_filename(unsigned int local, char* name);
unsigned char checksum(unsigned char start, unsigned char* data, int length);
unsigned int getclock(void);
void log_payload(int len, unsigned char source, unsigned int timestamp);
void log_xbee(unsigned char c, unsigned char source);
void log_pprz(unsigned char c, unsigned char source);
int do_log(void);

DirList list;
EmbeddedFileSystem efs;
EmbeddedFile filer;
EmbeddedFile filew;

unsigned char xbeel_payload[XBEE_PAYLOAD_LEN];
unsigned char pprzl_payload[PPRZ_PAYLOAD_LEN];
volatile unsigned char xbeel_payload_len;
volatile unsigned char pprzl_payload_len;
unsigned char xbeel_error;
unsigned char pprzl_error;
unsigned char log_buffer[MSG_SIZE]  __attribute__ ((aligned));
static unsigned int xbeel_timestamp = 0;
static unsigned int pprzl_timestamp = 0;
static unsigned int otf_timestamp = 0;
unsigned int nb_messages = 0;
unsigned int nb_fail_write = 0;
int bytes = 0;
unsigned int clock_msb = 0;
unsigned int clock_lsb_last = 0;
int log_run = 1;
int turb_received = 0;
unsigned char ac_id = 0;

/* SPI0 SLAVE */
#if 1
static void SPI0_ISR(void) __attribute__((naked));

/* S0SPCR settings */
#define S0SPCR_bit_enable  (0<<2)  /* 8 bits               */
#define S0SPCR_CPHA        (0<<3)  /* sample on first edge */
#define S0SPCR_CPOL        (0<<4)  /* clock idles low      */
#define S0SPCR_MSTR        (0<<5)  /* slave mode           */
#define S0SPCR_LSBF        (0<<6)  /* lsb first            */
#define S0SPCR_SPIE        (1<<7)  /* interrupt enable     */

#define S0SPCR_LSF_VAL (S0SPCR_bit_enable | S0SPCR_CPHA | \
			S0SPCR_CPOL | S0SPCR_MSTR | \
			S0SPCR_LSBF | S0SPCR_SPIE);

#define CPSDVSR 64

/* S0SPR bits */
#define ROVR 5
#define WCOL 6
#define SPIF 7
/* S0SPCR bits */
#define SPIE 7

#define PINSEL0_SCK  (1<<8)
#define PINSEL0_MISO (1<<10)
#define PINSEL0_MOSI (1<<12)
#define PINSEL0_SSEL (1<<14)

static inline void main_spi_init(void) {
  /* setup pins for SPI0 (SCK, MISO, MOSI, SS) */
  PINSEL0 |= PINSEL0_SCK | PINSEL0_MISO | PINSEL0_MOSI | PINSEL0_SSEL;

  S0SPCR = S0SPCR_LSF_VAL;
  S0SPCCR = CPSDVSR;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI0);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI0);     // SPI1 interrupt enabled
  VICVectCntl7 = VIC_ENABLE | VIC_SPI0;
  VICVectAddr7 = (uint32_t)SPI0_ISR;    // address of the ISR

  SetBit(S0SPCR, SPIE);
}

static void SPI0_ISR(void) {
  ISR_ENTRY();

  static uint8_t prev = 0;

  if ( bit_is_set(S0SPSR, SPIF)) { /* transfer complete  */
    uint8_t foo = S0SPDR;

    if ((foo == 0xF0) && (prev == 0x23)) log_run = 0;
    if ((foo == 0xF1) && (prev == 0x23)) log_run = 1;
    prev = foo;

    S0SPDR = 0x10 | log_run | turb_received;
  }

  /* clear_it */
  S0SPINT = 1<<SPI0IF;

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}
#endif

void set_filename(unsigned int local, char* name)
{
    /* do not use sprintf or similar */
    int i;

    for (i=7; i>=0; i--) {
        name[i] = (local % 10) + '0';
        local /= 10;
    }
    name[8]='.';name[9]='t';name[10]='l';name[11]='m';name[12]=0;
}

unsigned char checksum(unsigned char start, unsigned char* data, int length)
{
    int i;
    unsigned char retval = start;
    for (i=0;i<length;i++) retval += data[i];

    return retval;
}

unsigned int getclock(void)
{
    uint64_t clock;
    uint32_t clock_lsb;

    clock_lsb = T0TC;

    if (clock_lsb < clock_lsb_last) clock_msb++;
    clock_lsb_last = clock_lsb;

    clock = (((uint64_t)clock_msb << 32) | (uint64_t)clock_lsb) / LOG_DIV;

    return(clock & 0xFFFFFFFF);
}

/** Parsing a frame data and copy the payload to the log buffer */
void log_payload(int len, unsigned char source, unsigned int timestamp)
{
  unsigned char chk;

  /* start delimiter */
  log_buffer[0] = STX;

  /* length is just payload */
  log_buffer[1] = len & 0xFF;

  /* source */
  log_buffer[2] = source;

  /* add a 32bit timestamp */
  log_buffer[3] = (timestamp) & 0xFF;       // LSB first
  log_buffer[4] = (timestamp >> 8)  & 0xFF;
  log_buffer[5] = (timestamp >> 16) & 0xFF;
  log_buffer[6] = (timestamp >> 24) & 0xFF;

  /* data is already written */

  /* calculate checksum over start+length+timestamp+data */
  log_buffer[LOG_DATA_OFFSET+len] = checksum(0, &log_buffer[1], LOG_DATA_OFFSET+len-1);

  /* write data, start+length+timestamp+data+checksum */
  chk = file_write(&filew, LOG_DATA_OFFSET+len+1, log_buffer);

  if (len != chk)
  {
    nb_fail_write++;
  }

  bytes += chk;
  nb_messages++;
//  dl_parse_msg();
}

/** Parsing a XBee API frame */
void log_xbee(unsigned char c, unsigned char source)
{
  static unsigned char xbeel_status = XBEE_UNINIT;
  static unsigned char cs, payload_idx, i;
  static unsigned char ac_id_temp;

  switch (xbeel_status) {
  case XBEE_UNINIT:
    if (c == XBEE_START)
    {
// serial receive broken with MAX
#ifndef USE_MAX11040
      xbeel_timestamp = getclock();
#endif
      xbeel_status++;
    }
    break;
  case XBEE_GOT_START:
    xbeel_payload_len = c<<8;
    xbeel_status++;
    break;
  case XBEE_GOT_LENGTH_MSB:
    xbeel_payload_len |= c;
    xbeel_status++;
    payload_idx = 0;
    cs=0;
    break;
  case XBEE_GOT_LENGTH_LSB:
    xbeel_payload[payload_idx] = c;
    cs += c;
    payload_idx++;
    /* get aircraft id */
    if (payload_idx == 5)
      ac_id_temp = xbeel_payload[payload_idx];
    if (payload_idx == xbeel_payload_len)
      xbeel_status++;
    break;
  case XBEE_GOT_PAYLOAD:
    if (c + cs != 0xff)
      goto error;
    if ((xbeel_payload[0] != XBEE_RX16_ID) &&
        (xbeel_payload[0] != XBEE_TX16_ID))
      goto error;
    /* copy the XBee message to the logger buffer */
    for (i = 0; i < xbeel_payload_len-XBEE_RFDATA_OFFSET; i++) {
      log_buffer[i+LOG_DATA_OFFSET] = xbeel_payload[i+XBEE_RFDATA_OFFSET];
    }

    /* set aircraft id from first correct message */
    if (ac_id == 0)
      ac_id = ac_id_temp;

// serial receive broken with MAX
#ifndef USE_MAX11040
    log_payload(xbeel_payload_len-XBEE_RFDATA_OFFSET, source, xbeel_timestamp);
#endif
    LED_TOGGLE(LED_GREEN);
    goto restart;
  }
  return;
 error:
  xbeel_error++;
 restart:
  xbeel_status = XBEE_UNINIT;
  return;
}

void log_pprz(unsigned char c, unsigned char source)
{
  static unsigned char pprzl_status = UNINIT;
  static unsigned char _ck_a, _ck_b, payload_idx, i;

  switch (pprzl_status) {
  case UNINIT:
    if (c == STX)
// serial receive broken with MAX
#ifndef USE_MAX11040
      pprzl_timestamp = getclock();
#endif
      pprzl_status++;
    break;
  case GOT_STX:
    pprzl_payload_len = c-4; /* Counting STX, LENGTH and CRC1 and CRC2 */
    _ck_a = _ck_b = c;
    pprzl_status++;
    payload_idx = 0;
    break;
  case GOT_LENGTH:
    pprzl_payload[payload_idx] = c;
    _ck_a += c; _ck_b += _ck_a;
    payload_idx++;
    if (payload_idx == pprzl_payload_len)
      pprzl_status++;
    break;
  case GOT_PAYLOAD:
    if (c != _ck_a)
      goto error;
    pprzl_status++;
    break;
  case GOT_CRC1:
    if (c != _ck_b)
      goto error;
    /* copy the pprz message to the logger buffer */
    for (i = 0; i < pprzl_payload_len; i++) {
      log_buffer[i+LOG_DATA_OFFSET] = pprzl_payload[i];
    }
// serial receive broken with MAX
#ifndef USE_MAX11040
    log_payload(pprzl_payload_len, source, pprzl_timestamp);
#endif
    LED_TOGGLE(LED_GREEN);
    goto restart;
  }
  return;
 error:
  pprzl_error++;
 restart:
  pprzl_status = UNINIT;
  return;
}

void log_otf(unsigned char c, unsigned char source)
{
  static unsigned char otf_status = OTF_UNINIT, otf_idx = 0, otf_crs_idx;
  static char otf_inp[64];
  static unsigned int counter;
  static short course[3];
  static unsigned int altitude;
  static unsigned char checksum;
  static msg_count = 0;

  switch (otf_status) {

  case OTF_WAIT_START:
    if (c == OTF_START) {
      otf_status++;
      otf_idx = 0;
    } else {
      otf_status = OTF_UNINIT;
    }
    break;

  case OTF_WAIT_COUNTER:
    if (isdigit((int)c)) {
      if (otf_idx == 0) {
        otf_timestamp = getclock();
      }
      otf_inp[otf_idx++] = c;
    } else {
      if ((otf_idx > 4) && (c == OTF_LIMITER)) {
        otf_inp[otf_idx] = 0;
        counter = atoi(otf_inp);
        otf_idx = 0;
        otf_crs_idx = 0;
        otf_status++;
      } else {
        otf_status = OTF_UNINIT;
      }
    }
    break;

  case OTF_WAIT_ANGLES:
    if (isdigit((int)c) || (c == '-') || (c == '.')) {
      otf_inp[otf_idx++] = c;
    } else {
      if ((otf_idx > 1) && (otf_idx < 9) && (c == OTF_LIMITER)) {
        otf_inp[otf_idx] = 0;
        course[otf_crs_idx] = (int16_t) (100. * atof(otf_inp));
        otf_idx = 0;
        if (otf_crs_idx++ == 2) {
          otf_status++;
        }
      } else {
        otf_status = OTF_UNINIT;
      }
    }
    break;

  case OTF_WAIT_ALTITUDE:
    if (isdigit((int)c) || (c == '-') || (c == '.')) {
      otf_inp[otf_idx++] = c;
    } else {
      if ((otf_idx > 1) && (otf_idx < 9) && (c == OTF_LIMITER)) {
        otf_inp[otf_idx] = 0;
        altitude = (int32_t) (100. * atof(otf_inp));
        otf_idx = 0;
        otf_status++;
      } else {
        otf_status = OTF_UNINIT;
      }
    }
    break;

  case OTF_WAIT_CHECKSUM:
    if (isxdigit((int)c)) {
      otf_inp[otf_idx++] = c;
    } else {
      if ((otf_idx == 2) && (c == OTF_END)) {
        otf_inp[otf_idx] = 0;
        checksum = strtol(otf_inp, NULL, 16);
        otf_idx = 0;
/*
  <message name="FLOW_AP_OTF" id="179">
     <field name="counter"    type="uint32"/>
     <field name="velocity"   type="int16"  unit="cm/s" alt_unit="m/s"/>
     <field name="a_attack"   type="int16"  unit="centideg" alt_unit="deg"/>
     <field name="a_sidesl"   type="int16"  unit="centideg" alt_unit="deg"/>
     <field name="altitude"   type="int32"  unit="cm" alt_unit="m"/>
     <field name="checksum"   type="uint8"/>
  </message>
*/
/*
otf_timestamp = 0x000075DC;
counter = 0x12345678;
course[3];
altitude=0x12346;
checksum=0x23;
course[0]=0x2345;
course[1]=0x3456;
course[2]=0x4567;
*/
        log_buffer[LOG_DATA_OFFSET+0] = ac_id;  // sender_id
        log_buffer[LOG_DATA_OFFSET+1] = 179;    // message_id (FLOW_AP_OTF)

        log_buffer[LOG_DATA_OFFSET+2 + 0] = (counter      ) & 0xFF;
        log_buffer[LOG_DATA_OFFSET+2 + 1] = (counter >> 8 ) & 0xFF;
        log_buffer[LOG_DATA_OFFSET+2 + 2] = (counter >> 16) & 0xFF;
        log_buffer[LOG_DATA_OFFSET+2 + 3] = (counter >> 24) & 0xFF;

        log_buffer[LOG_DATA_OFFSET+2 + 4] = (course[0]     ) & 0xFF;
        log_buffer[LOG_DATA_OFFSET+2 + 5] = (course[0] >> 8) & 0xFF;
        log_buffer[LOG_DATA_OFFSET+2 + 6] = (course[1]     ) & 0xFF;
        log_buffer[LOG_DATA_OFFSET+2 + 7] = (course[1] >> 8) & 0xFF;
        log_buffer[LOG_DATA_OFFSET+2 + 8] = (course[2]     ) & 0xFF;
        log_buffer[LOG_DATA_OFFSET+2 + 9] = (course[2] >> 8) & 0xFF;

        log_buffer[LOG_DATA_OFFSET+2 + 10] = (altitude      ) & 0xFF;
        log_buffer[LOG_DATA_OFFSET+2 + 11] = (altitude >> 8 ) & 0xFF;
        log_buffer[LOG_DATA_OFFSET+2 + 12] = (altitude >> 16) & 0xFF;
        log_buffer[LOG_DATA_OFFSET+2 + 13] = (altitude >> 24) & 0xFF;
        log_buffer[LOG_DATA_OFFSET+2 + 14] = (checksum      ) & 0xFF;

        if (ac_id != 0) {
          turb_received = 2;
          log_payload(2 + 15, LOG_SOURCE_UART0, otf_timestamp);
        }
        else {
          /* wait 4s before starting with the default aircraft id */
          if (msg_count++ > 400) ac_id = DEFAULT_AC_ID;
        }
        LED_TOGGLE(LED_RED);
      }
      otf_status = OTF_UNINIT;
    }
    break;

  default:
    otf_status = OTF_UNINIT;
    break;
  }
}

int do_log(void)
{
    unsigned int count;
    unsigned char name[13];
    unsigned char inc;
    int temp;

	if(efs_init(&efs, 0) != 0) {
		return(-1);
	}

    /* find an unused file number the dumb way */
    for (count = 1; count < 0xFFFFFFF; count++)
    {
        set_filename(count, name);
        if(file_fopen(&filer, &efs.myFs, name,'r')!=0) break;
        file_fclose(&filer);
    }

    if (file_fopen(&filew, &efs.myFs, name, 'w' ) != 0)
    {
		return(-1);
    }

    /* write to SD until key is pressed */
    while (((IO0PIN & (1<<LOG_STOP_KEY))>>LOG_STOP_KEY) && (log_run == 1))
    {

#ifdef USE_MAX11040
      if ((max11040_data == MAX11040_DATA_AVAILABLE) &&
          (max11040_buf_in != max11040_buf_out)) {
//        LED_TOGGLE(LED_GREEN);
        int i;

        max11040_data = MAX11040_IDLE;

        log_buffer[LOG_DATA_OFFSET+0] = ac_id;  // sender_id
        log_buffer[LOG_DATA_OFFSET+1] = 61;     // message_id (DL_TURB_PRESSURE_RAW)

	while(max11040_buf_in != max11040_buf_out) {
          for (i=0; i<16; i++) {
            log_buffer[LOG_DATA_OFFSET+2 + i*4 + 0] = (max11040_values[max11040_buf_out][i]      ) & 0xFF;
            log_buffer[LOG_DATA_OFFSET+2 + i*4 + 1] = (max11040_values[max11040_buf_out][i] >> 8 ) & 0xFF;
            log_buffer[LOG_DATA_OFFSET+2 + i*4 + 2] = (max11040_values[max11040_buf_out][i] >> 16) & 0xFF;
            log_buffer[LOG_DATA_OFFSET+2 + i*4 + 3] = (max11040_values[max11040_buf_out][i] >> 24) & 0xFF;

          }
          log_payload(2 + 16 * 4, LOG_SOURCE_UART0, max11040_timestamp[max11040_buf_out]);
	  i = max11040_buf_out+1;
	  if (i >= MAX11040_BUF_SIZE) i=0;
          max11040_buf_out = i;
   	}
      }
#endif

#ifdef USE_UART0
        temp = 0;
        while (Uart0ChAvailable() && (temp++ < 128))
        {
			LED_TOGGLE(LED_GREEN);
			inc = Uart0Getch();
#ifdef LOG_XBEE
            log_xbee(inc, LOG_SOURCE_UART0);
#else
#ifdef LOG_PPRZ
            log_pprz(inc, LOG_SOURCE_UART0);
#else
#error no log transport protocol selected
#endif
#endif
        }
#endif
#ifdef USE_UART1
        temp = 0;
        while (Uart1ChAvailable() && (temp++ < 128))
        {
//			LED_TOGGLE(LED_GREEN);
			inc = Uart1Getch();
#ifdef LOG_OTF
            log_otf(inc, LOG_SOURCE_UART1);
#else
#ifdef LOG_XBEE
            log_xbee(inc, LOG_SOURCE_UART1);
#else
#ifdef LOG_PPRZ
            log_pprz(inc, LOG_SOURCE_UART1);
#else
#error no log transport protocol selected
#endif
#endif
#endif
        }
#endif
    }
    LED_OFF(LED_GREEN);

    file_fclose( &filew );
    fs_umount( &efs.myFs ) ;

    return 0;
}

int main(void)
{
  int waitloop, ledcount;
  main_init();

#ifdef _DEBUG_BOARD_
  while(1)
  {
    if (IO0PIN & (1 << LOG_STOP_KEY))
    {
      LED_ON(LED_YELLOW);
    }
    else
    {
      LED_OFF(LED_YELLOW);
    }

    if (IO1PIN & (1 << CARD_DETECT_PIN))
    {
      LED_OFF(LED_GREEN);
    }
    else
    {
      LED_ON(LED_GREEN);
    }

    if (IO0PIN & (1 << POWER_DETECT_PIN))
//    if (IO0PIN & (1 << VBUS_PIN))
    {
      LED_ON(LED_RED);
    }
    else
    {
      LED_OFF(LED_RED);
    }
  }
#endif


#ifdef _DEBUG_BOARD_
  while(1)
  {
    if (IO0PIN & (1 << LOG_STOP_KEY))
    {
      LED_ON(LED_YELLOW);
    }
    else
    {
      LED_OFF(LED_YELLOW);
    }

    if (IO1PIN & (1 << CARD_DETECT_PIN))
    {
      LED_OFF(LED_GREEN);
    }
    else
    {
      LED_ON(LED_GREEN);
    }

    if (IO0PIN & (1 << POWER_DETECT_PIN))
//    if (IO0PIN & (1 << VBUS_PIN))
    {
      LED_ON(LED_RED);
    }
    else
    {
      LED_OFF(LED_RED);
    }
  }
#endif

  // Direct SD Reader Mode
  if ((IO0PIN & _BV(VBUS_PIN))>>VBUS_PIN)
  {
    LED_OFF(LED_YELLOW);
    LED_ON(LED_RED);
    main_mass_storage();
  }

  while(1)
  {
    LED_ON(LED_YELLOW);
    do_log();
    LED_OFF(LED_YELLOW);

    waitloop = 0;
    ledcount = 0;

    while (waitloop < 20)
    {
      sys_time_usleep(100000);

      {
        if (ledcount++ > 9) {
          ledcount=0;
          LED_ON(LED_YELLOW);
        } else {
          LED_OFF(LED_YELLOW);
        }
        if (((IO0PIN & _BV(LOG_STOP_KEY))>>LOG_STOP_KEY) == 1) {
          waitloop=0;
        } else {
          waitloop++;
        }
      }

      if ((IO0PIN & _BV(VBUS_PIN))>>VBUS_PIN)
      {
        LED_OFF(LED_YELLOW);
        LED_ON(LED_RED);
        main_mass_storage();
      }
    }
    LED_ON(LED_YELLOW);
    while (((IO0PIN & _BV(LOG_STOP_KEY))>>LOG_STOP_KEY) == 0);
  }

  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_init();
  led_init();


#ifdef USE_MAX11040
  max11040_init_ssp();
  max11040_init();
#endif

#if 1
  main_spi_init();
#endif

  mcu_int_enable();

  PINSEL2 = ~ (0x0c);
}

static inline void main_periodic_task( void ) {
}
