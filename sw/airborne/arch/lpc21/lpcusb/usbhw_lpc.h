/*
	LPCUSB, an USB device driver for LPC microcontrollers
	Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright
	   notice, this list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright
	   notice, this list of conditions and the following disclaimer in the
	   documentation and/or other materials provided with the distribution.
	3. The name of the author may not be used to endorse or promote products
	   derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
	IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
	OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
	IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
	NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
	THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


/**
	Hardware definitions for the LPC214x USB controller

	These are private to the usbhw module
*/


/* Common LPC2148 definitions, related to USB */
#define	PCONP			*(volatile unsigned int *)0xE01FC0C4
#define	PLL1CON			*(volatile unsigned int *)0xE01FC0A0
#define	PLL1CFG			*(volatile unsigned int *)0xE01FC0A4
#define	PLL1STAT		*(volatile unsigned int *)0xE01FC0A8
#define	PLL1FEED		*(volatile unsigned int *)0xE01FC0AC

#define PINSEL0			*(volatile unsigned int *)0xE002C000
#define PINSEL1			*(volatile unsigned int *)0xE002C004
#define IOPIN0			*(volatile unsigned int *)0xE0028000
#define IOSET0			*(volatile unsigned int *)0xE0028004
#define IODIR0			*(volatile unsigned int *)0xE0028008
#define IOCLR0			*(volatile unsigned int *)0xE002800C

/* USB register definitions */
#define USBIntSt		*(volatile unsigned int *)0xE01FC1C0

#define USBDevIntSt		*(volatile unsigned int *)0xE0090000
#define USBDevIntEn		*(volatile unsigned int *)0xE0090004
#define USBDevIntClr	*(volatile unsigned int *)0xE0090008
#define USBDevIntSet	*(volatile unsigned int *)0xE009000C
#define USBDevIntPri	*(volatile unsigned int *)0xE009002C

#define USBEpIntSt		*(volatile unsigned int *)0xE0090030
#define USBEpIntEn		*(volatile unsigned int *)0xE0090034
#define USBEpIntClr		*(volatile unsigned int *)0xE0090038
#define USBEpIntSet		*(volatile unsigned int *)0xE009003C
#define USBEpIntPri		*(volatile unsigned int *)0xE0090040

#define USBReEP			*(volatile unsigned int *)0xE0090044
#define USBEpInd		*(volatile unsigned int *)0xE0090048
#define USBMaxPSize		*(volatile unsigned int *)0xE009004C

#define USBRxData		*(volatile unsigned int *)0xE0090018
#define USBRxPLen		*(volatile unsigned int *)0xE0090020
#define USBTxData		*(volatile unsigned int *)0xE009001C
#define USBTxPLen		*(volatile unsigned int *)0xE0090024
#define USBCtrl			*(volatile unsigned int *)0xE0090028

#define USBCmdCode		*(volatile unsigned int *)0xE0090010
#define USBCmdData		*(volatile unsigned int *)0xE0090014

/* USBIntSt bits */
#define USB_INT_REQ_LP				(1<<0)
#define USB_INT_REQ_HP				(1<<1)
#define USB_INT_REQ_DMA				(1<<2)
#define USB_need_clock				(1<<8)
#define EN_USB_BITS					(1<<31)

/* USBDevInt... bits */
#define FRAME						(1<<0)
#define EP_FAST						(1<<1)
#define EP_SLOW						(1<<2)
#define DEV_STAT					(1<<3)
#define CCEMTY						(1<<4)
#define CDFULL						(1<<5)
#define RxENDPKT					(1<<6)
#define TxENDPKT					(1<<7)
#define EP_RLZED					(1<<8)
#define ERR_INT						(1<<9)

/* USBRxPLen bits */
#define PKT_LNGTH					(1<<0)
#define PKT_LNGTH_MASK				0x3FF
#define DV							(1<<10)
#define PKT_RDY						(1<<11)

/* USBCtrl bits */
#define RD_EN						(1<<0)
#define WR_EN						(1<<1)
#define LOG_ENDPOINT				(1<<2)

/* protocol engine command codes */
	/* device commands */
#define CMD_DEV_SET_ADDRESS			0xD0
#define CMD_DEV_CONFIG				0xD8
#define CMD_DEV_SET_MODE			0xF3
#define CMD_DEV_READ_CUR_FRAME_NR	0xF5
#define CMD_DEV_READ_TEST_REG		0xFD
#define CMD_DEV_STATUS				0xFE		/* read/write */
#define CMD_DEV_GET_ERROR_CODE		0xFF
#define CMD_DEV_READ_ERROR_STATUS	0xFB
	/* endpoint commands */
#define CMD_EP_SELECT				0x00
#define CMD_EP_SELECT_CLEAR			0x40
#define CMD_EP_SET_STATUS			0x40
#define CMD_EP_CLEAR_BUFFER			0xF2
#define CMD_EP_VALIDATE_BUFFER		0xFA

/* set address command */
#define DEV_ADDR					(1<<0)
#define DEV_EN						(1<<7)

/* configure device command */
#define CONF_DEVICE					(1<<0)

/* set mode command */
#define AP_CLK						(1<<0)
#define INAK_CI						(1<<1)
#define INAK_CO						(1<<2)
#define INAK_II						(1<<3)
#define INAK_IO						(1<<4)
#define INAK_BI						(1<<5)
#define INAK_BO						(1<<6)

/* set get device status command */
#define CON							(1<<0)
#define CON_CH						(1<<1)
#define SUS							(1<<2)
#define SUS_CH						(1<<3)
#define RST							(1<<4)

/* get error code command */
// ...

/* Select Endpoint command read bits */
#define EPSTAT_FE					(1<<0)
#define EPSTAT_ST					(1<<1)
#define EPSTAT_STP					(1<<2)
#define EPSTAT_PO					(1<<3)
#define EPSTAT_EPN					(1<<4)
#define EPSTAT_B1FULL				(1<<5)
#define EPSTAT_B2FULL				(1<<6)

/* CMD_EP_SET_STATUS command */
#define EP_ST						(1<<0)
#define EP_DA						(1<<5)
#define EP_RF_MO					(1<<6)
#define EP_CND_ST					(1<<7)

/* read error status command */
#define PID_ERR						(1<<0)
#define UEPKT						(1<<1)
#define DCRC						(1<<2)
#define TIMEOUT						(1<<3)
#define EOP							(1<<4)
#define B_OVRN						(1<<5)
#define BTSTF						(1<<6)
#define TGL_ERR						(1<<7)








