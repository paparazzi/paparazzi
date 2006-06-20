/*
	(c) 2006, Bertrik Sikken, bertrik@sikken.nl

	definitions of structure of standard USB packets
*/

#ifndef _USBSTRUCT_H_
#define _USBSTRUCT_H_


#include "type.h"


/*
	setup packet definitions
*/
typedef struct {
	U8	bmRequestType;
	U8	bRequest;
	U16	wValue;
	U16	wIndex;
	U16	wLength;
} TSetupPacket;


#define EP_XFER_CONTROL			0
#define EP_XFER_ISOC			1
#define EP_XFER_BULK			2
#define EP_XFER_INT				3

#define REQTYPE_GET_DIR(x)		(((x)>>7)&0x01)
#define REQTYPE_GET_TYPE(x)		(((x)>>5)&0x03)
#define REQTYPE_GET_RECIP(x)	((x)&0x1F)

#define REQTYPE_DIR_TO_DEVICE	0
#define REQTYPE_DIR_TO_HOST		1

#define REQTYPE_TYPE_STANDARD	0
#define REQTYPE_TYPE_CLASS		1
#define REQTYPE_TYPE_VENDOR		2
#define REQTYPE_TYPE_RESERVED	3

#define REQTYPE_RECIP_DEVICE	0
#define REQTYPE_RECIP_INTERFACE	1
#define REQTYPE_RECIP_ENDPOINT	2
#define REQTYPE_RECIP_OTHER		3

/* standard requests */
#define	REQ_GET_STATUS			0x00
#define REQ_CLEAR_FEATURE		0x01
#define REQ_SET_FEATURE			0x03
#define REQ_SET_ADDRESS			0x05
#define REQ_GET_DESCRIPTOR		0x06
#define REQ_SET_DESCRIPTOR		0x07
#define REQ_GET_CONFIGURATION	0x08
#define REQ_SET_CONFIGURATION	0x09
#define REQ_GET_INTERFACE		0x0A
#define REQ_SET_INTERFACE		0x0B
#define REQ_SYNCH_FRAME			0x0C

#define CLASS_PER_INTERFACE		0x00
#define CLASS_COMM				0x02
#define CLASS_HID				0x03
#define CLASS_PHYSICAL			0x05
#define CLASS_STILL_IMAGE		0x06
#define CLASS_PRINTER			0x07
#define CLASS_MASS_STORAGE		0x08
#define CLASS_HUB				0x09
#define CLASS_CDC_DATA			0x0A
#define CLASS_CSCID				0x0B
#define CLASS_CONTENT_SEC		0x0D
#define CLASS_VIDEO				0x0E
#define CLASS_APP_SPEC			0xFE
#define CLASS_VENDOR_SPEC		0xFF

/* class requests HID */
#define HID_GET_REPORT			0x01
#define HID_GET_IDLE			0x02
#define HID_GET_PROTOCOL	 	0x03
#define HID_SET_REPORT			0x09
#define HID_SET_IDLE			0x0A
#define HID_SET_PROTOCOL		0x0B

/* class requests mass storage */
#define MSC_GET_MAX_LUN			0xFE
#define MSC_BULK_RESET			0xFF

/* feature selectors */
#define FEA_ENDPOINT_HALT		0x00
#define FEA_REMOTE_WAKEUP		0x01
#define FEA_TEST_MODE			0x02

/*
	USB descriptors
*/

typedef struct {
	U8	bLength;
	U8	bDescriptorType;
} TUSBDescHeader;

#define DESC_DEVICE				1
#define DESC_CONFIGURATION		2
#define DESC_STRING				3
#define DESC_INTERFACE			4
#define DESC_ENDPOINT			5
#define DESC_DEVICE_QUALIFIER	6
#define DESC_OTHER_SPEED		7
#define DESC_INTERFACE_POWER	8

#define DESC_HID_HID			0x21
#define DESC_HID_REPORT			0x22
#define DESC_HID_PHYSICAL		0x23

#define GET_DESC_TYPE(x)		(((x)>>8)&0xFF)
#define GET_DESC_INDEX(x)		((x)&0xFF)

#endif /* _USBSTRUCT_H_ */

