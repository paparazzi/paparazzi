/*****************************************************************************\
*              efs - General purpose Embedded Filesystem library              *
*          --------------------- -----------------------------------          *
*                                                                             *
* Filename : lpc2000_spi.c                                                     *
* Description : This  contains the functions needed to use efs for        *
*               accessing files on an SD-card connected to an LPC2xxx.        *
*                                                                             *
* This library is free software; you can redistribute it and/or               *
* modify it under the terms of the GNU Lesser General Public                  *
* License as published by the Free Software Foundation; either                *
* version 2.1 of the License, or (at your option) any later version.          *
*                                                                             *
* This library is distributed in the hope that it will be useful,             *
* but WITHOUT ANY WARRANTY; without even the implied warranty of              *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU           *
* Lesser General Public License for more details.                             *
*                                                                             *
*                                                    (c)2005 Martin Thomas    *
*                                                                             *
\*****************************************************************************/

/*
	2006, Bertrik Sikken, modified for LPCUSB
*/


/*****************************************************************************/
#include "type.h"
#include "usbdebug.h"

#include "spi.h"
/*****************************************************************************/

/* General Purpose Input/Output (GPIO) */
#define IOPIN0         (*((volatile unsigned long *) 0xE0028000))
#define IOSET0         (*((volatile unsigned long *) 0xE0028004))
#define IODIR0         (*((volatile unsigned long *) 0xE0028008))
#define IOCLR0         (*((volatile unsigned long *) 0xE002800C))
#define IOPIN1         (*((volatile unsigned long *) 0xE0028010))
#define IOSET1         (*((volatile unsigned long *) 0xE0028014))
#define IODIR1         (*((volatile unsigned long *) 0xE0028018))
#define IOCLR1         (*((volatile unsigned long *) 0xE002801C))

/* Pin Connect Block */
#define PINSEL0        (*((volatile unsigned long *) 0xE002C000))
#define PINSEL1        (*((volatile unsigned long *) 0xE002C004))
#define PINSEL2        (*((volatile unsigned long *) 0xE002C014))

/* SSP Controller */
#define SSPCR0         (*((volatile unsigned short* ) 0xE0068000))
#define SSPCR1         (*((volatile unsigned char * ) 0xE0068004))
#define SSPDR          (*((volatile unsigned short* ) 0xE0068008))
#define SSPSR          (*((volatile unsigned char * ) 0xE006800C))
#define SSPCPSR        (*((volatile unsigned char * ) 0xE0068010))
#define SSPIMSC        (*((volatile unsigned char * ) 0xE0068014))
#define SSPRIS         (*((volatile unsigned char * ) 0xE0068018))
#define SSPMIS         (*((volatile unsigned char * ) 0xE006801C))
#define SSPICR         (*((volatile unsigned char * ) 0xE0068020))
#define SSPDMACR       (*((volatile unsigned char * ) 0xE0068024))

// SSPCR0  Bit-Definitions
#define CPOL    6
#define CPHA    7
// SSPCR1  Bit-Defintions
#define SSE     1
#define MS      2
#define SCR     8
// SSPSR  Bit-Definitions
#define TNF     1
#define RNE     2
#define BSY     4

#define SPI_IODIR   IODIR0
#define SPI_IOSET   IOSET0

#define SPI_SCK_PIN    17   /* Clock       P0.17  out */
#define SPI_MISO_PIN   18   /* from Card   P0.18  in  */
#define SPI_MOSI_PIN   19   /* to Card     P0.19  out */
/* Card-Select P0.20 - GPIO out during startup */
#define SPI_SS_PIN     20

#define SPI_PINSEL     PINSEL1
#define SPI_SCK_FUNCBIT   2
#define SPI_MISO_FUNCBIT  4
#define SPI_MOSI_FUNCBIT  6
#define SPI_SS_FUNCBIT    8

#define SPI_PRESCALE_REG  SSPCPSR

/* only needed during init: */
#define SELECT_CARD()   IOCLR0 = (1<<SPI_SS_PIN)
#define UNSELECT_CARD()	IOSET0 = (1<<SPI_SS_PIN)

/*****************************************************************************/

/*****************************************************************************/

// Utility-functions which does not toggle CS.
// Only needed during card-init. During init
// the automatic chip-select is disabled for SSP

static U8 my_SPISend(U8 outgoing)
{
	while( !(SSPSR & (1<<TNF)) ) { ; }
	SSPDR = outgoing;
	while( !(SSPSR & (1<<RNE)) ) { ; }
	return SSPDR;
}

/*****************************************************************************/

void SPISetSpeed(U8 speed)
{
	speed &= 0xFE;
	if (speed < SPI_PRESCALE_MIN) {
		speed = SPI_PRESCALE_MIN;
	}
	SPI_PRESCALE_REG = speed;
}


void SPIInit(void)
{
	U8 i;

	DBG("spiInit for SPI(1)\n");

	// setup GPIO
	SPI_IODIR |= (1 << SPI_SCK_PIN) | (1 << SPI_MOSI_PIN) | (1 << SPI_SS_PIN);
	SPI_IODIR &= ~(1 << SPI_MISO_PIN);

	// set Chip-Select high - unselect card
	UNSELECT_CARD();

	// setup Pin-Functions - keep automatic CS disabled during init
	SPI_PINSEL |= ( (2<<SPI_SCK_FUNCBIT) | (2<<SPI_MISO_FUNCBIT) |
		(2<<SPI_MOSI_FUNCBIT) | (0<<SPI_SS_FUNCBIT) );
	// enable SPI-Master - fastest speed
	SSPCR0 = ((8-1)<<0) | (0<<CPOL) | (0x1<<SCR);
	SSPCR1 = (1<<SSE);

	// low speed during init
	SPISetSpeed(254);

	/* Send 20 spi commands with card not selected */
	for (i = 0; i < 21; i++) {
		my_SPISend(0xff);
	}

	// enable automatic slave CS for SSP
	SSPCR1 &= ~(1<<SSE); // disable interface
	SPI_PINSEL |= ( (2<<SPI_SCK_FUNCBIT) | (2<<SPI_MISO_FUNCBIT) |
		(2<<SPI_MOSI_FUNCBIT) | (2<<SPI_SS_FUNCBIT) );
	SSPCR1 |= (1<<SSE); // enable interface

	// SPI clock 15MHz @60MHz PCLK
	SPISetSpeed(2);
}

/*****************************************************************************/

/*****************************************************************************/

U8 SPISend(U8 outgoing)
{
	while( !(SSPSR & (1<<TNF)) ) ;
	SSPDR = outgoing;
	while( !(SSPSR & (1<<RNE)) ) ;
	return SSPDR;
}


void SPISendN(U8 * pbBuf, int iLen)
{
	int i;
	U8 temp;
	for (i = 0; i < iLen; i++) {
        while( !(SSPSR & (1<<TNF)) ) ;
        SSPDR = pbBuf[i];
        while( !(SSPSR & (1<<RNE)) ) ;
        temp = SSPDR;
	}
}


void SPIRecvN(U8 * pbBuf, int iLen)
{
	int i;

	for (i = 0; i < iLen; i++) {
        while( !(SSPSR & (1<<TNF)) ) ;
        SSPDR = 0xFF;
        while( !(SSPSR & (1<<RNE)) ) ;
		pbBuf[i] = SSPDR;
	}
}

/*****************************************************************************/
