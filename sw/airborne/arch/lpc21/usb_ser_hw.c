/*
	LPCUSB, an USB device driver for LPC microcontrollers
	Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)
    adapted to pprz    Martin Mueller (martinmm@pfump.org)

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

/*
	Minimal implementation of a USB serial port, using the CDC class.
	This example application simply echoes everything it receives right back
	to the host.

	Windows:
	Extract the usbser.sys file from .cab file in C:\WINDOWS\Driver Cache\i386
	and store it somewhere (C:\temp is a good place) along with the usbser.inf
	file. Then plug in the LPC214x and direct windows to the usbser driver.
	Windows then creates an extra COMx port that you can open in a terminal
	program, like hyperterminal.

	Linux:
	The device should be recognised automatically by the cdc_acm driver,
	which creates a /dev/ttyACMx device file that acts just like a regular
	serial port.

*/


#include <string.h>
#include "std.h"
#include <stdbool.h>
#include "LPC21xx.h"
#include "armVIC.h"
#include "mcu_periph/usb_serial.h"
#include BOARD_CONFIG

#include "lpcusb/usbapi.h"

#if USE_USB_SERIAL
#if PCLK < 18000000
#error PCLK needs to be higher than 18MHz for USB to work properly
#endif
#endif

#ifndef USB_VIC_SLOT
#define USB_VIC_SLOT 10
#endif

#define INT_IN_EP               0x81
#define BULK_OUT_EP             0x05
#define BULK_IN_EP              0x82

#define MAX_PACKET_SIZE         64

#define LE_WORD(x)              ((x)&0xFF),((x)>>8)

// CDC definitions
#define CS_INTERFACE            0x24
#define CS_ENDPOINT             0x25

#define	SET_LINE_CODING         0x20
#define	GET_LINE_CODING         0x21
#define	SET_CONTROL_LINE_STATE  0x22

#define VCOM_FIFO_SIZE          128

#define EOF                     (-1)
#define ASSERT(x)

typedef struct {
    int         head;
    int         tail;
    uint8_t     *buf;
} fifo_t;

// data structure for GET_LINE_CODING / SET_LINE_CODING class requests
typedef struct {
    uint32_t    dwDTERate;
    uint8_t     bCharFormat;
    uint8_t     bParityType;
    uint8_t     bDataBits;
} TLineCoding;

int allow_line_coding = 0;
 /* this settings are virtual unless you enable line coding */
static TLineCoding LineCoding = {115200, 0, 0, 8};
static uint8_t abBulkBuf[64];
static uint8_t abClassReqData[8];

static uint8_t txdata[VCOM_FIFO_SIZE];
static uint8_t rxdata[VCOM_FIFO_SIZE];

static fifo_t txfifo;
static fifo_t rxfifo;

static bool BulkOut_is_blocked = false;

// forward declaration of interrupt handler
static void USBIntHandler(void) __attribute__ ((interrupt("IRQ")));

static void BulkOut(U8 bEP, U8 bEPStatus);

#ifdef USE_USB_LINE_CODING
void set_linecoding(TLineCoding linecoding);
#endif

void fifo_init(fifo_t *fifo, U8 *buf);
BOOL fifo_put(fifo_t *fifo, U8 c);
BOOL fifo_get(fifo_t *fifo, U8 *pc);
int  fifo_avail(fifo_t *fifo);
int	 fifo_free(fifo_t *fifo);

static const uint8_t abDescriptors[] = {

// device descriptor
	0x12,
	DESC_DEVICE,
	LE_WORD(0x0101),			// bcdUSB
	0x02,						// bDeviceClass
	0x00,						// bDeviceSubClass
	0x00,						// bDeviceProtocol
	MAX_PACKET_SIZE0,			// bMaxPacketSize
	LE_WORD(0x7070),			// idVendor
	LE_WORD(0x1235),			// idProduct
	LE_WORD(0x0100),			// bcdDevice
	0x01,						// iManufacturer
	0x02,						// iProduct
	0x03,						// iSerialNumber
	0x01,						// bNumConfigurations

// configuration descriptor
	0x09,
	DESC_CONFIGURATION,
	LE_WORD(67),				// wTotalLength
	0x02,						// bNumInterfaces
	0x01,						// bConfigurationValue
	0x00,						// iConfiguration
	0xC0,						// bmAttributes
	0x32,						// bMaxPower
// control class interface
	0x09,
	DESC_INTERFACE,
	0x00,						// bInterfaceNumber
	0x00,						// bAlternateSetting
	0x01,						// bNumEndPoints
	0x02,						// bInterfaceClass
	0x02,						// bInterfaceSubClass
	0x01,						// bInterfaceProtocol, linux requires value of 1 for the cdc_acm module
	0x00,						// iInterface
// header functional descriptor
	0x05,
	CS_INTERFACE,
	0x00,
	LE_WORD(0x0110),
// call management functional descriptor
	0x05,
	CS_INTERFACE,
	0x01,
	0x01,						// bmCapabilities = device handles call management
	0x01,						// bDataInterface
// ACM functional descriptor
	0x04,
	CS_INTERFACE,
	0x02,
	0x02,						// bmCapabilities
// union functional descriptor
	0x05,
	CS_INTERFACE,
	0x06,
	0x00,						// bMasterInterface
	0x01,						// bSlaveInterface0
// notification EP
	0x07,
	DESC_ENDPOINT,
	INT_IN_EP,					// bEndpointAddress
	0x03,						// bmAttributes = intr
	LE_WORD(8),					// wMaxPacketSize
	0xFE,						// bInterval
// data class interface descriptor
	0x09,
	DESC_INTERFACE,
	0x01,						// bInterfaceNumber
	0x00,						// bAlternateSetting
	0x02,						// bNumEndPoints
	0x0A,						// bInterfaceClass = data
	0x00,						// bInterfaceSubClass
	0x00,						// bInterfaceProtocol
	0x00,						// iInterface
// data EP OUT
	0x07,
	DESC_ENDPOINT,
	BULK_OUT_EP,				// bEndpointAddress
	0x02,						// bmAttributes = bulk
	LE_WORD(MAX_PACKET_SIZE),	// wMaxPacketSize
	0x00,						// bInterval
// data EP in
	0x07,
	DESC_ENDPOINT,
	BULK_IN_EP,					// bEndpointAddress
	0x02,						// bmAttributes = bulk
	LE_WORD(MAX_PACKET_SIZE),	// wMaxPacketSize
	0x00,						// bInterval

	// string descriptors
	0x04,
	DESC_STRING,
	LE_WORD(0x0409),

	0x0E,
	DESC_STRING,
	'L', 0, 'P', 0, 'C', 0, 'U', 0, 'S', 0, 'B', 0,

	0x14,
	DESC_STRING,
	'U', 0, 'S', 0, 'B', 0, 'S', 0, 'e', 0, 'r', 0, 'i', 0, 'a', 0, 'l', 0,

	0x12,
	DESC_STRING,
	'1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0,

// terminating zero
	0
};


void fifo_init(fifo_t *fifo, U8 *buf)
{
	fifo->head = 0;
	fifo->tail = 0;
	fifo->buf = buf;
}

BOOL fifo_put(fifo_t *fifo, U8 c)
{
	int next;

	// check if FIFO has room
	next = (fifo->head + 1) % VCOM_FIFO_SIZE;
	if (next == fifo->tail) {
		// full
		return FALSE;
	}

	fifo->buf[fifo->head] = c;
	fifo->head = next;

	return TRUE;
}

BOOL fifo_get(fifo_t *fifo, U8 *pc)
{
	int next;

	// check if FIFO has data
	if (fifo->head == fifo->tail) {
		return FALSE;
	}

	next = (fifo->tail + 1) % VCOM_FIFO_SIZE;

	*pc = fifo->buf[fifo->tail];
	fifo->tail = next;

	return TRUE;
}

int fifo_avail(fifo_t *fifo)
{
	return (VCOM_FIFO_SIZE + fifo->head - fifo->tail) % VCOM_FIFO_SIZE;
}

int fifo_free(fifo_t *fifo)
{
	return (VCOM_FIFO_SIZE - 1 - fifo_avail(fifo));
}

#ifdef USE_USB_LINE_CODING
void set_linecoding(TLineCoding linecoding)
{
    uint16_t baud;
    uint8_t mode;

    // set the baudrate
    baud = (uint16_t)((PCLK / ((linecoding.dwDTERate) * 16.0)) + 0.5);

    // set the number of characters and other
    // user specified operating parameters
    switch (linecoding.bCharFormat)
    {
        case 0: /* 1 stop bit */
            mode = ULCR_STOP_1;
            break;
        case 1: /* 1.5 stop bit (only with 5 bit character) */
        case 2: /* 2 stop bit */
            mode = ULCR_STOP_2;
            break;
        default:
            mode = ULCR_STOP_1;
            break;
    }
    switch (linecoding.bParityType)
    {
        case 0:  mode += ULCR_PAR_NO;
            break;
        case 1:  mode += ULCR_PAR_ODD;
            break;
        case 2:  mode += ULCR_PAR_EVEN;
            break;
        case 3:  mode += ULCR_PAR_MARK;
            break;
        case 4:  mode += ULCR_PAR_SPACE;
            break;
        default: mode += ULCR_PAR_NO;
            break;
    }
    switch (linecoding.bDataBits)
    {
        case 5:  mode += ULCR_CHAR_5;
            break;
        case 6:  mode += ULCR_CHAR_6;
            break;
        case 7:  mode += ULCR_CHAR_7;
            break;
        case 8:  mode += ULCR_CHAR_8;
            break;
        case 16:
        default: mode += ULCR_CHAR_8;
            break;
    }

#if USE_UART0
    U0LCR = ULCR_DLAB_ENABLE;             // select divisor latches
    U0DLL = (uint8_t)baud;                // set for baud low byte
    U0DLM = (uint8_t)(baud >> 8);         // set for baud high byte
    U0LCR = (mode & ~ULCR_DLAB_ENABLE);
#endif
#if USE_UART1
    U1LCR = ULCR_DLAB_ENABLE;             // select divisor latches
    U1DLL = (uint8_t)baud;                // set for baud low byte
    U1DLM = (uint8_t)(baud >> 8);         // set for baud high byte
    U1LCR = (mode & ~ULCR_DLAB_ENABLE);
#endif
}
#endif

#ifdef USE_USB_LINE_CODING
void VCOM_allow_linecoding(uint8_t mode)
{
    allow_line_coding = mode;
}
#endif

/**
	Writes one character to VCOM port

	@param [in] c character to write
	@returns character written, or EOF if character could not be written
 */
int VCOM_putchar(int c)
{
	return fifo_put(&txfifo, c) ? c : EOF;
}

/**
	Reads one character from VCOM port

	@returns character read, or EOF if character could not be read
 */
int VCOM_getchar(void)
{
  int result;
  U8 c;

  result = fifo_get(&rxfifo, &c) ? c : EOF;

  if (BulkOut_is_blocked && fifo_free(&rxfifo) >= MAX_PACKET_SIZE) {
    disableIRQ();
    // get more data from usb bus
    BulkOut(BULK_OUT_EP, 0);
    BulkOut_is_blocked = false;
    enableIRQ();
  }

  return result;
}

/**
	Checks if buffer free in VCOM buffer

	@returns TRUE if len bytes are free
 */
bool_t VCOM_check_free_space(uint8_t len)
{
	return (fifo_free(&txfifo) >= len ? TRUE : FALSE);
}


/**
	Checks if data available in VCOM buffer

	@returns character read, or EOF if character could not be read
 */
int VCOM_check_available(void)
{
	return (fifo_avail(&rxfifo));
}


/**
	Local function to handle incoming bulk data

	@param [in] bEP
	@param [in] bEPStatus
 */
static void BulkOut(U8 bEP, U8 bEPStatus __attribute__((unused)))
{
	int i, iLen;

	if (fifo_free(&rxfifo) < MAX_PACKET_SIZE) {
	  // may not fit into fifo
	  BulkOut_is_blocked = true;
	  return;
	}

	// get data from USB into intermediate buffer
	iLen = USBHwEPRead(bEP, abBulkBuf, sizeof(abBulkBuf));
	for (i = 0; i < iLen; i++) {
		// put into FIFO
		if (!fifo_put(&rxfifo, abBulkBuf[i])) {
			// overflow... :(
			ASSERT(FALSE);
			break;
		}
	}
}


/**
	Local function to handle outgoing bulk data

	@param [in] bEP
	@param [in] bEPStatus
 */
static void BulkIn(U8 bEP, U8 bEPStatus __attribute__((unused)))
{
	int i, iLen;

	if (fifo_avail(&txfifo) == 0) {
		// no more data, disable further NAK interrupts until next USB frame
		USBHwNakIntEnable(0);
		return;
	}

	// get bytes from transmit FIFO into intermediate buffer
	for (i = 0; i < MAX_PACKET_SIZE; i++) {
		if (!fifo_get(&txfifo, &abBulkBuf[i])) {
			break;
		}
	}
	iLen = i;

	// send over USB
	if (iLen > 0) {
		USBHwEPWrite(bEP, abBulkBuf, iLen);
	}
}


/**
	Local function to handle the USB-CDC class requests

	@param [in] pSetup
	@param [out] piLen
	@param [out] ppbData
 */
static BOOL HandleClassRequest(TSetupPacket *pSetup, int *piLen, U8 **ppbData)
{
	switch (pSetup->bRequest) {

	// set line coding
	case SET_LINE_CODING:
		memcpy((U8 *)&LineCoding, *ppbData, 7);
		*piLen = 7;
#ifdef USE_USB_LINE_CODING
        if (allow_line_coding)
        {
            set_linecoding(LineCoding);
        }
#endif
		break;

	// get line coding
	case GET_LINE_CODING:
		*ppbData = (U8 *)&LineCoding;
		*piLen = 7;
		break;

	// set control line state
	case SET_CONTROL_LINE_STATE:
		// bit0 = DTR, bit1 = RTS
		break;

	default:
		return FALSE;
	}
	return TRUE;
}


/**
	Interrupt handler

	Simply calls the USB ISR, then signals end of interrupt to VIC
 */
static void USBIntHandler(void)
{
	USBHwISR();
	VICVectAddr = 0x00;    // dummy write to VIC to signal end of ISR
}


static void USBFrameHandler(U16 wFrame __attribute__((unused)))
{
	if (fifo_avail(&txfifo) > 0) {
		// data available, enable NAK interrupt on bulk in
		USBHwNakIntEnable(INACK_BI);
	}
}

// Periph with generic device API
struct usb_serial_periph usb_serial;

// Functions for the generic device API
static int usb_serial_check_free_space(struct usb_serial_periph* p __attribute__((unused)), uint8_t len)
{
  return (int)VCOM_check_free_space(len);
}

static void usb_serial_transmit(struct usb_serial_periph* p __attribute__((unused)), uint8_t byte)
{
  VCOM_putchar(byte);
}

static void usb_serial_send(struct usb_serial_periph* p __attribute__((unused))) { }

// Empty for lpc21
void VCOM_event(void) {}

// Empty for lpc21
void VCOM_send_message(void) {}

void VCOM_init(void) {
	// initialise stack
	USBInit();
#ifdef USE_USB_LINE_CODING
	// set default line coding
    set_linecoding(LineCoding);
#endif

	// register descriptors
	USBRegisterDescriptors(abDescriptors);

	// register class request handler
	USBRegisterRequestHandler(REQTYPE_TYPE_CLASS, HandleClassRequest, abClassReqData);

	// register endpoint handlers
	USBHwRegisterEPIntHandler(INT_IN_EP, NULL);
	USBHwRegisterEPIntHandler(BULK_IN_EP, BulkIn);
	USBHwRegisterEPIntHandler(BULK_OUT_EP, BulkOut);

	// register frame handler
	USBHwRegisterFrameHandler(USBFrameHandler);

	// enable bulk-in interrupts on NAKs
	USBHwNakIntEnable(INACK_BI);

	// initialise fifos
	fifo_init(&txfifo, txdata);
	fifo_init(&rxfifo, rxdata);

	// set up USB interrupt
	VICIntSelect &= ~VIC_BIT(VIC_USB);               // select IRQ for USB
	VICIntEnable = VIC_BIT(VIC_USB);

	_VIC_CNTL(USB_VIC_SLOT) = VIC_ENABLE | VIC_USB;
	_VIC_ADDR(USB_VIC_SLOT) = (uint32_t)USBIntHandler;

	// connect to bus
	USBHwConnect(TRUE);

  // Configure generic device
  usb_serial.device.periph = (void *)(&usb_serial);
  usb_serial.device.check_free_space = (check_free_space_t) usb_serial_check_free_space;
  usb_serial.device.transmit = (transmit_t) usb_serial_transmit;
  usb_serial.device.send_message = (send_message_t) usb_serial_send;
}
