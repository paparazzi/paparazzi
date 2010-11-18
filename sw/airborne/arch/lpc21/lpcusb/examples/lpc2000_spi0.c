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

/* SPI0 (Serial Peripheral Interface 0) */
#define S0SPCR			*(volatile unsigned int *)0xE0020000
#define S0SPSR			*(volatile unsigned int *)0xE0020004
#define S0SPDR			*(volatile unsigned int *)0xE0020008
#define S0SPCCR			*(volatile unsigned int *)0xE002000C
#define S0SPTCR			*(volatile unsigned int *)0xE0020010
#define S0SPTSR			*(volatile unsigned int *)0xE0020014
#define S0SPTOR			*(volatile unsigned int *)0xE0020018
#define S0SPINT			*(volatile unsigned int *)0xE002001C


// SP0SPCR  Bit-Definitions
#define CPHA    3
#define CPOL    4
#define MSTR    5
// SP0SPSR  Bit-Definitions
#define SPIF    7

#define SPI_IODIR      IODIR0
#define SPI_IOSET      IOSET0
#define SPI_PINSEL     PINSEL0

#define SPI_SCK_PIN    4    /* Clock       P0.4  out */
#define SPI_MISO_PIN   5    /* from Card   P0.5  in  */
#define SPI_MOSI_PIN   6    /* to Card     P0.6  out */
#define SPI_SS_PIN	   7    /* Card-Select P0.7 - GPIO out */

#define SPI_SCK_FUNCBIT   8
#define SPI_MISO_FUNCBIT  10
#define SPI_MOSI_FUNCBIT  12
#define SPI_SS_FUNCBIT    14

#define SPI_PRESCALE_REG  S0SPCCR
#define SPI_PRESCALE_MIN  8

#define SELECT_CARD()   IOCLR0 = (1<<SPI_SS_PIN)
#define UNSELECT_CARD()	IOSET0 = (1<<SPI_SS_PIN)

/*****************************************************************************/

/*****************************************************************************/

// Utility-functions which does not toggle CS.
// Only needed during card-init.

static U8 my_SPISend(U8 outgoing)
{
	S0SPDR = outgoing;
	while( !(S0SPSR & (1<<SPIF)) ) { ; }

	return S0SPDR;
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

/*****************************************************************************/

void SPIInit(void)
{
	U8 i;

	DBG("spiInit for SPI(0)\n");

	// setup GPIO
	SPI_IODIR |= (1 << SPI_SCK_PIN) | (1 << SPI_MOSI_PIN) | (1 << SPI_SS_PIN);
	SPI_IODIR &= ~(1 << SPI_MISO_PIN);

	// set Chip-Select high - unselect card
	UNSELECT_CARD();

	SPI_PINSEL |= ( (1<<SPI_SCK_FUNCBIT) | (1<<SPI_MISO_FUNCBIT) |
		(1<<SPI_MOSI_FUNCBIT) );
	// enable SPI-Master
	S0SPCR = (1<<MSTR)|(0<<CPOL);

	// low speed during init
	SPISetSpeed(254);

	/* Send 20 spi commands with card not selected */
	for (i = 0; i < 21; i++) {
		my_SPISend(0xff);
	}

	// SPI clock 15MHz @60MHz PCLK
	SPISetSpeed(2);
}

/*****************************************************************************/

/*****************************************************************************/

U8 SPISend(U8 outgoing)
{
	U8 incoming;

	SELECT_CARD();
	S0SPDR = outgoing;
	while( !(S0SPSR & (1<<SPIF)) ) ;
	incoming = S0SPDR;
	UNSELECT_CARD();

	return incoming;
}


void SPISendN(U8 * pbBuf, int iLen)
{
	int i;
	U8 temp;

	SELECT_CARD();
	for (i = 0; i < iLen; i++) {
        S0SPDR = pbBuf[i];
    	while( !(S0SPSR & (1<<SPIF)) ) ;
        temp = S0SPDR;
	}
	UNSELECT_CARD();
}


void SPIRecvN(U8 * pbBuf, int iLen)
{
	int i;

	SELECT_CARD();
	for (i = 0; i < iLen; i++) {
        S0SPDR = 0xFF;
    	while( !(S0SPSR & (1<<SPIF)) ) ;
		pbBuf[i] = S0SPDR;
	}
	UNSELECT_CARD();
}

/*****************************************************************************/
