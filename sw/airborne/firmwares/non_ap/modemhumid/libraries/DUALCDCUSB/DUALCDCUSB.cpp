

/* Copyright (c) 2011, Peter Barrett  
**  
** Permission to use, copy, modify, and/or distribute this software for  
** any purpose with or without fee is hereby granted, provided that the  
** above copyright notice and this permission notice appear in all copies.  
** 
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL  
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED  
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR  
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES  
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,  
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,  
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS  
** SOFTWARE.  
*/

/* adapted to support 2nd CDC USB port by Martin Mueller */

#include <util/atomic.h>
#include "USBAPI.h"
#include "DUALCDCUSB.h"

#if defined(USBCON)

typedef struct
{
	u32	dwDTERate;
	u8	bCharFormat;
	u8 	bParityType;
	u8 	bDataBits;
	u8	lineState;
} LineInfo;

static volatile LineInfo _usbLineInfo = { 57600, 0x00, 0x00, 0x00, 0x00 };
static volatile int32_t breakValue = -1;

DUALCDCUSB_ DualCDCUSB;

bool DUALCDCUSB_::setup(USBSetup& setup)
{
  u8 r = setup.bRequest;
  u8 requestType = setup.bmRequestType;

  if (REQUEST_DEVICETOHOST_CLASS_INTERFACE == requestType) {
    if (CDC_GET_LINE_CODING == r) {
      USB_SendControl(0,(void*)&_usbLineInfo,7);
      return true;
    }
  }

  if (REQUEST_HOSTTODEVICE_CLASS_INTERFACE == requestType) {
    if (CDC_SEND_BREAK == r) {
      breakValue = ((uint16_t)setup.wValueH << 8) | setup.wValueL;
    }

    if (CDC_SET_LINE_CODING == r) {
      USB_RecvControl((void*)&_usbLineInfo,7);
    }

    if (CDC_SET_CONTROL_LINE_STATE == r) {
      _usbLineInfo.lineState = setup.wValueL;
    }

    if (CDC_SET_LINE_CODING == r || CDC_SET_CONTROL_LINE_STATE == r) {
      /* nothing to do here */
    }
    return true;
  }
  return false;
}

void DUALCDCUSB_::get_status(unsigned char* buf)
{
}

int DUALCDCUSB_::getInterface(uint8_t* interfaceCount)
{
  *interfaceCount += 2;

  CDCDescriptor _cdcInterface2 = {
    /* Interface Association */
    D_IAD(2,2,CDC_COMMUNICATION_INTERFACE_CLASS,CDC_ABSTRACT_CONTROL_MODEL,1),

    /* CDC communication interface */
    D_INTERFACE(CDC_ACM_INTERFACE2,1,CDC_COMMUNICATION_INTERFACE_CLASS,CDC_ABSTRACT_CONTROL_MODEL,0),
    /* Header (1.10 bcd) */
    D_CDCCS(CDC_HEADER,0x10,0x01),
    /* Device handles call management (not) */
    D_CDCCS(CDC_CALL_MANAGEMENT,1,1),
    /* SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE supported */
    D_CDCCS4(CDC_ABSTRACT_CONTROL_MANAGEMENT,6),
    /* Communication interface is master, data interface is slave 0 */
    D_CDCCS(CDC_UNION,CDC_ACM_INTERFACE2,CDC_DATA_INTERFACE2),
    D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_ACM2),USB_ENDPOINT_TYPE_INTERRUPT,0x10,0x40),

    /* CDC data interface */
    D_INTERFACE(CDC_DATA_INTERFACE2,2,CDC_DATA_INTERFACE_CLASS,0,0),
    D_ENDPOINT(USB_ENDPOINT_OUT(CDC_ENDPOINT_OUT2),USB_ENDPOINT_TYPE_BULK,USB_EP_SIZE,0),
    D_ENDPOINT(USB_ENDPOINT_IN(CDC_ENDPOINT_IN2),USB_ENDPOINT_TYPE_BULK,USB_EP_SIZE,0)
  };
  return USB_SendControl(0, &_cdcInterface2, sizeof(_cdcInterface2));
}

int DUALCDCUSB_::getDescriptor(USBSetup& setup __attribute__((unused)))
{
  return 0;
}

uint8_t DUALCDCUSB_::getShortName(char* name)
{
  memcpy(name, "Paparazzi", 9);
  return 9;
}

bool DUALCDCUSB_::begin(void)
{
}

DUALCDCUSB_::DUALCDCUSB_(void) : PluggableUSBModule(3, 2, epType)
{
  epType[0] = EP_TYPE_INTERRUPT_IN;
  epType[1] = EP_TYPE_BULK_OUT;
  epType[2] = EP_TYPE_BULK_IN;
  PluggableUSB().plug(this);
}

void Serial__::begin(unsigned long /* baud_count */)
{
	peek_buffer = -1;
}

void Serial__::begin(unsigned long /* baud_count */, byte /* config */)
{
	peek_buffer = -1;
}

void Serial__::end(void)
{
}

int Serial__::available(void)
{
	if (peek_buffer >= 0) {
		return 1 + USB_Available(CDC_RX2);
	}
	return USB_Available(CDC_RX2);
}

int Serial__::peek(void)
{
	if (peek_buffer < 0)
		peek_buffer = USB_Recv(CDC_RX2);
	return peek_buffer;
}

int Serial__::read(void)
{
	if (peek_buffer >= 0) {
		int c = peek_buffer;
		peek_buffer = -1;
		return c;
	}
	return USB_Recv(CDC_RX2);
}

int Serial__::availableForWrite(void)
{
	return USB_SendSpace(CDC_TX2);
}

void Serial__::flush(void)
{
	USB_Flush(CDC_TX2);
}

size_t Serial__::write(uint8_t c)
{
	return write(&c, 1);
}

size_t Serial__::write(const uint8_t *buffer, size_t size)
{
	/* only try to send bytes if the high-level CDC connection itself 
	 is open (not just the pipe) - the OS should set lineState when the port
	 is opened and clear lineState when the port is closed.
	 bytes sent before the user opens the connection or after
	 the connection is closed are lost - just like with a UART. */
	
	// TODO - ZE - check behavior on different OSes and test what happens if an
	// open connection isn't broken cleanly (cable is yanked out, host dies
	// or locks up, or host virtual serial port hangs)
	if (_usbLineInfo.lineState > 0)	{
		int r = USB_Send(CDC_TX2|TRANSFER_RELEASE,buffer,size);
		if (r > 0) {
			return r;
		} else {
			setWriteError();
			return 0;
		}
	}
	setWriteError();
	return 0;
}

// This operator is a convenient way for a sketch to check whether the
// port has actually been configured and opened by the host (as opposed
// to just being connected to the host).  It can be used, for example, in 
// setup() before printing to ensure that an application on the host is
// actually ready to receive and display the data.
// We add a short delay before returning to fix a bug observed by Federico
// where the port is configured (lineState != 0) but not quite opened.
Serial__::operator bool() {
	bool result = false;
	if (_usbLineInfo.lineState > 0) 
		result = true;
	delay(10);
	return result;
}

unsigned long Serial__::baud() {
	// Disable interrupts while reading a multi-byte value
	uint32_t baudrate;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		baudrate =  _usbLineInfo.dwDTERate;
	}
	return baudrate;
}

uint8_t Serial__::stopbits() {
	return _usbLineInfo.bCharFormat;
}

uint8_t Serial__::paritytype() {
	return _usbLineInfo.bParityType;
}

uint8_t Serial__::numbits() {
	return _usbLineInfo.bDataBits;
}

bool Serial__::dtr() {
	return _usbLineInfo.lineState & 0x1;
}

bool Serial__::rts() {
	return _usbLineInfo.lineState & 0x2;
}

int32_t Serial__::readBreak() {
	int32_t ret;
	// Disable IRQs while reading and clearing breakValue to make
	// sure we don't overwrite a value just set by the ISR.
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		ret = breakValue;
		breakValue = -1;
	}
	return ret;
}

Serial__ Serial2;


#endif /* USBCON */
