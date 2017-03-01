
#ifndef DUALCDCUSB_H
#define DUALCDCUSB_H

#include <stdint.h>
#include <Arduino.h>

#if ARDUINO < 10606
#error DUALCDCUSB needs Arduino IDE 1.6.6 or higher
#endif

#if !defined(USBCON)
#error DUALCDCUSB needs an USB MCU
#endif

#if defined(ARDUINO_ARCH_AVR)
#include "PluggableUSB.h"
#else
#include "USB/PluggableUSB.h"
#endif

#if defined(__SAMD21G18A__)
#define USB_SendControl         USBDevice.sendControl
#define USB_Available           USBDevice.available
#define USB_Recv                USBDevice.recv
#define USB_Send                USBDevice.send
#define USB_Flush               USBDevice.flush
#define is_write_enabled(x)     (1)
#endif
#define CDC_ENDPOINT_ACM2 (4)
#define CDC_ENDPOINT_OUT2 (5)
#define CDC_ENDPOINT_IN2  (6)

#define CDC_ACM_INTERFACE2   2	// CDC ACM
#define CDC_DATA_INTERFACE2  3	// CDC Data

#define CDC_RX2 CDC_ENDPOINT_OUT2
#define CDC_TX2 CDC_ENDPOINT_IN2

#ifndef SERIAL_BUFFER_SIZE
#if ((RAMEND - RAMSTART) < 1023)
#define SERIAL_BUFFER_SIZE 16
#else
#define SERIAL_BUFFER_SIZE 64
#endif
#endif
#if (SERIAL_BUFFER_SIZE>256)
#error Please lower the CDC Buffer size
#endif

_Pragma("pack(1)")



class Serial__ : public Stream
{
private:
	int peek_buffer;
public:
	Serial__() { peek_buffer = -1; };
	void begin(unsigned long);
	void begin(unsigned long, uint8_t);
	void end(void);

	virtual int available(void);
	virtual int peek(void);
	virtual int read(void);
	int availableForWrite(void);
	virtual void flush(void);
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t*, size_t);
	using Print::write; // pull in write(str) and write(buf, size) from Print
	operator bool();

	volatile uint8_t _rx_buffer_head;
	volatile uint8_t _rx_buffer_tail;
	unsigned char _rx_buffer[SERIAL_BUFFER_SIZE];

	// This method allows processing "SEND_BREAK" requests sent by
	// the USB host. Those requests indicate that the host wants to
	// send a BREAK signal and are accompanied by a single uint16_t
	// value, specifying the duration of the break. The value 0
	// means to end any current break, while the value 0xffff means
	// to start an indefinite break.
	// readBreak() will return the value of the most recent break
	// request, but will return it at most once, returning -1 when
	// readBreak() is called again (until another break request is
	// received, which is again returned once).
	// This also mean that if two break requests are received
	// without readBreak() being called in between, the value of the
	// first request is lost.
	// Note that the value returned is a long, so it can return
	// 0-0xffff as well as -1.
	int32_t readBreak();

	// These return the settings specified by the USB host for the
	// serial port. These aren't really used, but are offered here
	// in case a sketch wants to act on these settings.
	uint32_t baud();
	uint8_t stopbits();
	uint8_t paritytype();
	uint8_t numbits();
	bool dtr();
	bool rts();
	enum {
		ONE_STOP_BIT = 0,
		ONE_AND_HALF_STOP_BIT = 1,
		TWO_STOP_BITS = 2,
	};
	enum {
		NO_PARITY = 0,
		ODD_PARITY = 1,
		EVEN_PARITY = 2,
		MARK_PARITY = 3,
		SPACE_PARITY = 4,
	};

};
extern Serial__ Serial2;

typedef struct {
  InterfaceDescriptor DualCDCInterface;
} dual_cdc_usb_Descriptor;

_Pragma("pack()")

class DUALCDCUSB_ : public PluggableUSBModule {

protected:
  int getInterface(uint8_t* interfaceNum);
  int getDescriptor(USBSetup& setup);
  bool setup(USBSetup& setup);
  uint8_t getShortName(char* name);

public:
  bool begin(void);
  void get_status(unsigned char* buf);
  DUALCDCUSB_(void);

private:
  uint8_t epType[3];
};

extern DUALCDCUSB_ DualCDCUSB;

#endif	/* DUALCDCUSB_H */
