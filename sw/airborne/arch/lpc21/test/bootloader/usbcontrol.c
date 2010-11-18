/*
	LPCUSB, an USB device driver for LPC microcontrollers
	Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
	Control transfer handler.

	In case of a control-write (host-to-device), this module collects the full
	control message in a local buffer, then sends it off to an installed
	request handler.

	In case of a control-read (device-to-host), an installed request handler
	is asked to handle the request and provide return data. The handler can
	either put the data in the control data buffer through the supplied pointer,
	or it can supply a new data pointer. In both cases, the handler is required
	to update the data length in *piLen;


	Currently, control transfers are handled in a very simple way, keeping
	almost no state about the control transfer progress (setup stage, data
	stage, status stage). We simply follow the host: if it sends data, we store
	it in the control data buffer and if it asks for data, we just send the next
	block.

*/

#include "type.h"
#include "usbdebug.h"

#include "usbstruct.h"
#include "usbapi.h"


// maximum total size of control transfer data
#define	MAX_CONTROL_SIZE	128

#define	MAX_REQ_HANDLERS	4	// standard, class, vendor, reserved

static TSetupPacket		Setup;
static U8				abControlData[MAX_CONTROL_SIZE];

static U8				*pbData;	// pointer to data buffer
static int				iResidue;		// remaining bytes in buffer
static int				iLen;		// total length of control transfer

static TFnHandleRequest *apfnReqHandlers[4] = {NULL, NULL, NULL, NULL};
static TFnHandleRequest *apfnPreReqHandler = NULL;


/*************************************************************************
	_PreHandleRequest
	=================
		Local function to handle a request before data arrived by calling
		a special installed request handler.

	Called in case of data going from host to device, the handler can
	update the data pointer *ppbData.

	IN		pSetup		The setup packet
	IN/OUT	*piLen		Pointer to data length
			ppbData		Data buffer.

	Returns TRUE if the request was handles successfully
**************************************************************************/
static BOOL _PreHandleRequest(TSetupPacket *pSetup, int *piLen, U8 **ppbData)
{
	TFnHandleRequest *pfnHandler;

	pfnHandler = apfnPreReqHandler;
	if (pfnHandler == NULL) {
		// no pre-handler installed
		return FALSE;
	}

	return pfnHandler(pSetup, piLen, ppbData);
}


/*************************************************************************
	_HandleRequest
	==============
		Local function to handle a request by calling one of the installed
		request handlers.

	In case of data going from host to device, the data is at *ppbData.
	In case of data going from device to host, the handler can either
	choose to write its data at *ppbData or update the data pointer.

	IN		pSetup		The setup packet
	IN/OUT	*piLen		Pointer to data length
			ppbData		Data buffer.

	Returns TRUE if the request was handles successfully
**************************************************************************/
static BOOL _HandleRequest(TSetupPacket *pSetup, int *piLen, U8 **ppbData)
{
	TFnHandleRequest *pfnHandler;
	int iType;

	iType = REQTYPE_GET_TYPE(pSetup->bmRequestType);
	pfnHandler = apfnReqHandlers[iType];
	if (pfnHandler == NULL) {
		DBG("No handler for reqtype %d\n", iType);
		return FALSE;
	}

	return pfnHandler(pSetup, piLen, ppbData);
}


/*************************************************************************
	StallControlPipe
	================
		Local function to stall the control endpoint

**************************************************************************/
static void StallControlPipe(U8 bEPStat)
{
	U8	*pb;
	int	i;

	DBG("STALL on [");
// dump setup packet
	pb = (U8 *)&Setup;
	for (i = 0; i < 8; i++) {
		DBG(" %02x", *pb++);
	}
	DBG("] stat=%x\n", bEPStat);
	USBHwEPStall(0x80, TRUE);
}


/*************************************************************************
	DataIn
	======
		Sends next chunk of data (possibly 0 bytes) to host

**************************************************************************/
static void DataIn(void)
{
	int iChunk;

	iChunk = MIN(MAX_PACKET_SIZE0, iResidue);
	USBHwEPWrite(0x80, pbData, iChunk);
	pbData += iChunk;
	iResidue -= iChunk;
}


/*************************************************************************
	USBHandleControlTransfer
	========================
		Handles IN/OUT transfers on EP0

**************************************************************************/
void USBHandleControlTransfer(U8 bEP, U8 bEPStat)
{
	int iChunk;

	if (bEP == 0x00) {
		// OUT transfer
		if (bEPStat & EP_STATUS_SETUP) {
			// setup packet, reset request message state machine
			USBHwEPRead(0x00, (U8 *)&Setup, &iLen);
//mmmm			DBG("S%x", Setup.bRequest);

			// defaults for data pointer and residue
			pbData = abControlData;
			iResidue = Setup.wLength;
			iLen = Setup.wLength;

			// ask installed handler to pre process request
			if (!_PreHandleRequest(&Setup, &iLen, &pbData)) {
				// silently ignore non installed handler
			}

			if ((Setup.wLength == 0) ||
				(REQTYPE_GET_DIR(Setup.bmRequestType) == REQTYPE_DIR_TO_HOST)) {
				// ask installed handler to process request
				if (!_HandleRequest(&Setup, &iLen, &pbData)) {
					DBG("_HandleRequest1 failed\n");
					StallControlPipe(bEPStat);
					return;
				}
				// send smallest of requested and offered length
				iResidue = MIN(iLen, Setup.wLength);
				// send first part (possibly a zero-length status message)
				DataIn();
			}

			if ((Setup.wLength != 0) &&
                (REQTYPE_GET_DIR(Setup.bmRequestType) == REQTYPE_DIR_TO_DEVICE)) {
				if (!_PreHandleRequest(&Setup, &iLen, &pbData)) {
					// this is not a must, might fail
				}
			}
		}
		else {
			if (iResidue > 0) {
				// store data
				iChunk = 0;
				USBHwEPRead(0x00, pbData, &iChunk);
				pbData += iChunk;
				iResidue -= iChunk;
				if (iResidue == 0) {
					// received all, send data to handler
					// TODO set pointer correctly
					pbData = abControlData;
					if (!_HandleRequest(&Setup, &iLen, &pbData)) {
						StallControlPipe(bEPStat);
						DBG("_HandleRequest2 failed\n");
						return;
					}
					// send status to host
					DataIn();
				}
			}
			else {
				// absorb zero-length status message
				USBHwEPRead(0x00, NULL, &iChunk);
				DBG(iChunk > 0 ? "?" : "");
			}
		}
	}
	else if (bEP == 0x80) {
		// IN transfer
		// send more data if available (possibly a 0-length packet)
		DataIn();
	}
	else {
		ASSERT(FALSE);
	}
}


/*************************************************************************
	USBRegisterPreRequestHandler
	============================
		Registers a callback for pre-handling requests

	IN		*pfnHandler	Callback function pointer

**************************************************************************/
void USBRegisterPreRequestHandler(TFnHandleRequest *pfnHandler)
{
	apfnPreReqHandler = pfnHandler;
}


/*************************************************************************
	USBRegisterRequestHandler
	=========================
		Registers a callback for handling requests

	IN		iType		Type of request, e.g. REQTYPE_TYPE_STANDARD
			*pfnHandler	Callback function pointer

**************************************************************************/
void USBRegisterRequestHandler(int iType, TFnHandleRequest *pfnHandler)
{
	ASSERT(iType >= 0);
	ASSERT(iType < 4);
	apfnReqHandlers[iType] = pfnHandler;
}

