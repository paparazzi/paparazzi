/*
 * Copyright (C) 2011 The Paparazzi Team
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
 */

/**
 * @file arch/lpc21/mcu_periph/spi_slave_hs_arch.c
 * @ingroup lpc21_arch
 *
 * Highspeed SPI Slave Interface.
 */

#include "spi_slave_hs_arch.h"
#include "mcu_periph/spi.h"

#include BOARD_CONFIG
#include "std.h"
#include "mcu.h"
#include "led.h"
#include "LPC21xx.h"
#include "ssp_hw.h"
#include "pprz_debug.h"
#include "armVIC.h"

struct spi_slave_hs spi_slave_hs;

/* High Speed SPI Slave Circular Buffer */
uint16_t spi_slave_hs_rx_insert_idx, spi_slave_hs_rx_extract_idx;
uint8_t spi_slave_hs_rx_buffer[SPI_SLAVE_HS_RX_BUFFER_SIZE];
uint8_t spi_slave_hs_tx_insert_idx, spi_slave_hs_tx_extract_idx;
uint8_t spi_slave_hs_tx_buffer[SPI_SLAVE_HS_TX_BUFFER_SIZE];

/* Prototypes */
static void SSP_ISR(void) __attribute__((naked));

/* SSPCR0 settings */
#define SSP_DDS   0x07 << 0  /* data size         : 8 bits                    */
//#define SSP_DDS   0x0F << 0  /* data size         : 16 bits                    */
#define SSP_FRF   0x00 << 4  /* frame format      : SPI                       */
#define SSP_CPOL  0x00 << 6  /* clock polarity    : data captured on first clock transition */
#define SSP_CPHA  0x00 << 7  /* clock phase       : SCK idles low             */
#define SSP_SCR   0x00 << 8  /* serial clock rate : divide by 1               */

#define SSPCR0_VAL  (SSP_DDS  |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR )

/* SSPCR1 settings */
#define SSP_LBM   0x00 << 0  /* loopback mode     : disabled                      */
#define SSP_SSE   0x00 << 1  /* SSP enable        : enable later when init ready  */
#define SSP_MS    0x01 << 2  /* master slave mode : slave                         */
#define SSP_SOD   0x00 << 3  /* slave output disable : don't care when master     */

#define SSPCR1_VAL  (SSP_LBM   |  SSP_SSE | SSP_MS   | SSP_SOD )

/* SSPCPSR settings
 * min value as master: 2
 * min value as slave: 12
 */
#if (PCLK == 15000000)
#define CPSDVSR    12
#else

#if (PCLK == 30000000)
#define CPSDVSR    24
#else

#if (PCLK == 60000000)
#define CPSDVSR    28
#else

#error unknown PCLK frequency
#endif
#endif
#endif

#define SSP_PINSEL1_SCK  (2<<2)
#define SSP_PINSEL1_MISO (2<<4)
#define SSP_PINSEL1_MOSI (2<<6)
#define SSP_PINSEL1_SSEL (2<<8)


#define SSP_Write(X)  SSPDR=(X)
#define SSP_Read()  SSPDR
#define SSP_Status()  SSPSR

/** default initial settings */
#ifndef SPI1_VIC_SLOT
#define SPI1_VIC_SLOT 7
#endif


// Functions for the generic device API
static int spi_slave_hs_check_free_space(struct spi_slave_hs *p __attribute__((unused)), uint8_t len __attribute__((unused)))
{
  return TRUE;
}

static void spi_slave_hs_transmit(struct spi_slave_hs *p __attribute__((unused)), uint8_t byte)
{
  uint8_t temp = (spi_slave_hs_tx_insert_idx + 1) % SPI_SLAVE_HS_TX_BUFFER_SIZE;
  if (temp != spi_slave_hs_tx_extract_idx)  /* there is room left */
  {
    spi_slave_hs_tx_buffer[spi_slave_hs_tx_insert_idx] = byte;
    spi_slave_hs_tx_insert_idx = temp;
  }
}

static void spi_slave_hs_send(struct spi_slave_hs *p __attribute__((unused))) { }

static int spi_slave_hs_char_available(struct spi_slave_hs *p __attribute__((unused)))
{
  return spi_slave_hs_rx_insert_idx != spi_slave_hs_rx_extract_idx;
}

static uint8_t spi_slave_hs_getch(struct spi_slave_hs *p __attribute__((unused)))
{
  uint8_t ret = spi_slave_hs_rx_buffer[spi_slave_hs_rx_extract_idx];
  spi_slave_hs_rx_extract_idx = (spi_slave_hs_rx_extract_idx + 1)%SPI_SLAVE_HS_RX_BUFFER_SIZE;
  return ret;
}

void spi_slave_hs_init(void)
{

  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= SSP_PINSEL1_SCK  | SSP_PINSEL1_MISO | SSP_PINSEL1_MOSI | SSP_PINSEL1_SSEL;

  /* setup SSP */
  // Control Registers
  SSPCR0 = SSPCR0_VAL;
  SSPCR1 = SSPCR1_VAL;
  // Clock Prescale Registers
  SSPCPSR = CPSDVSR;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);               /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT(VIC_SPI1);                 /* enable it            */
  _VIC_CNTL(SPI1_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
  _VIC_ADDR(SPI1_VIC_SLOT) = (uint32_t)SSP_ISR;      /* address of the ISR   */


  // Enable SPI Slave
  SetBit(SSPCR1, SSE);

  // Enable Receive interrupt
  SetBit(SSPIMSC, RXIM);

  // Configure generic device
  spi_slave_hs.device.periph = (void *)(&spi_slave_hs);
  spi_slave_hs.device.check_free_space = (check_free_space_t) spi_slave_hs_check_free_space;
  spi_slave_hs.device.put_byte = (put_byte_t) spi_slave_hs_transmit;
  spi_slave_hs.device.send_message = (send_message_t) spi_slave_hs_send;
  spi_slave_hs.device.char_available = (char_available_t) spi_slave_hs_char_available;
  spi_slave_hs.device.get_byte = (get_byte_t) spi_slave_hs_getch;

}

/*
 *  SSP Status:
 *
 *  ROVR  Read Overrun
 *  WCOL  Write Collision   (send new byte during a transfer in progress
 *  ABRT  SSEL inactive before end of transfer
 *
 *
 */


static void SSP_ISR(void)
{
  ISR_ENTRY();

  //LED_TOGGLE(3);

  // If any TX bytes are pending
  if (spi_slave_hs_tx_insert_idx != spi_slave_hs_tx_extract_idx) {
    uint8_t ret = spi_slave_hs_tx_buffer[spi_slave_hs_tx_extract_idx];
    spi_slave_hs_tx_extract_idx = (spi_slave_hs_tx_extract_idx + 1) % SPI_SLAVE_HS_TX_BUFFER_SIZE;
    SSP_Write(ret);
  } else {
    SSP_Write(0x00);
  }


  //do
  {
    uint16_t temp;

    // calc next insert index & store character
    temp = (spi_slave_hs_rx_insert_idx + 1) % SPI_SLAVE_HS_RX_BUFFER_SIZE;
    spi_slave_hs_rx_buffer[ spi_slave_hs_rx_insert_idx] = SSP_Read();

    // check for more room in queue
    if (temp !=  spi_slave_hs_rx_extract_idx) {
      spi_slave_hs_rx_insert_idx = temp;  // update insert index
    }

    // else overrun
  }
  // while FIFO not empty
  //while (SSPSR & RNE);

  /*
    // loop until not more interrupt sources
    while (((iid = U0IIR) & UIIR_NO_INT) == 0)
          while (U0LSR & ULSR_THRE)
            {
            // check if more data to send
            if (uart0_tx_insert_idx != uart0_tx_extract_idx)
              {
              U0THR = uart0_tx_buffer[uart0_tx_extract_idx];
              uart0_tx_extract_idx++;
              uart0_tx_extract_idx %= UART0_TX_BUFFER_SIZE;
              }
            else
              {
              // no
              uart0_tx_running = 0;       // clear running flag
              break;
              }
            }

  */
  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

