/*
	(c) 2006, Bertrik Sikken, bertrik@sikken.nl
*/


#include "type.h"
#include "usbstruct.h"		// for TSetupPacket

/*************************************************************************
	USB configuration
**************************************************************************/

#define MAX_PACKET_SIZE0	64

/*************************************************************************
	USB hardware interface
**************************************************************************/

// endpoint status sent through callback
#define EP_STATUS_DATA		(1<<0)		// EP has data
#define EP_STATUS_STALLED	(1<<1)		// EP is stalled
#define EP_STATUS_SETUP		(1<<2)		// EP received setup packet
#define EP_STATUS_NACKED	(1<<3)		// EP sent NAK
#define EP_STATUS_ERROR		(1<<4)		// EP data was overwritten by setup packet

// device status send through callback
#define DEV_STATUS_CONNECT		(1<<0)
#define DEV_STATUS_SUSPEND		(1<<2)
#define DEV_STATUS_RESET		(1<<4)

BOOL USBHwInit			(void);
void USBHwReset			(void);
void USBHwISR			(void);

void USBHwConnect		(BOOL fConnect);

void USBHwSetAddress	(U8 addr);
void USBHwConfigDevice	(BOOL fConfigured);

// endpoint operations
BOOL USBHwEPRead		(U8 bEP, U8 *pbBuf, int *piLen);
BOOL USBHwEPWrite		(U8 bEP, U8 *pbBuf, int iLen);
void USBHwEPStall		(U8 bEP, BOOL fStall);
BOOL USBHwGetEPStall	(U8 bEP);

// register a callback for endpoint events
typedef void (TFnEPIntHandler)	(U8 bEP, U8 bEPStatus);
void USBHwRegisterEPIntHandler	(U8 bEP, U16 wMaxPacketSize, TFnEPIntHandler *pfnHandler);

// register a callback for device status events
typedef void (TFnDevIntHandler)	(U8 bDevStatus);
void USBHwRegisterDevIntHandler	(TFnDevIntHandler *pfnHandler);

// register a callback for frame event
typedef void (TFnFrameHandler)(U16 wFrame);
void USBHwRegisterFrameHandler(TFnFrameHandler *pfnHandler);


/*************************************************************************
	USB application interface
**************************************************************************/

// initialise the complete stack, including HW
BOOL USBInit(void);

// register a callback for requests (vendor, class)
typedef BOOL (TFnHandleRequest)(TSetupPacket *pSetup, int *piLen, U8 **ppbData);
void USBRegisterRequestHandler(int iType, TFnHandleRequest *pfnHandler);
void USBRegisterPreRequestHandler(TFnHandleRequest *pfnHandler);

// register a callback for descriptors
typedef BOOL (TFnGetDescriptor)(U16 wTypeIndex, U16 wLangID, int *piLen, U8 **ppbData);
void USBRegisterDescriptorHandler(TFnGetDescriptor *pfnGetDesc);

// default standard request handler
BOOL USBHandleStandardRequest(TSetupPacket *pSetup, int *piLen, U8 **ppbData);

// default EP0 handler
void USBHandleControlTransfer(U8 bEP, U8 bEPStat);

// default descriptor handler
void USBRegisterDescriptors(const U8 *pabDescriptors);
BOOL USBHandleDescriptor(U16 wTypeIndex, U16 wLangID, int *piLen, U8 **ppbData);

