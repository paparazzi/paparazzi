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
	Standard request handler.

	This modules handles the 'chapter 9' processing, specifically the
	standard device requests in table 9-3 from the universal serial bus
	specification revision 2.0

	Specific types of devices may specify additional requests (for example
	HID devices add a GET_DESCRIPTOR request for interfaces), but they
	will not be part of this module.
*/

// TODO some requests have to return a request error if device not configured:
// TODO GET_INTERFACE, GET_STATUS, SET_INTERFACE, SYNCH_FRAME
// TODO this applies to the following if endpoint != 0:
// TODO SET_FEATURE, GET_FEATURE

#include "type.h"
#include "usbdebug.h"
#include "usbstruct.h"
#include "usbapi.h"

#define MAX_DESC_HANDLERS	4		// device, interface, endpoint, other

// device state info
static U8				bConfiguration = 0;
static TFnGetDescriptor	*pfnGetDescriptor = NULL;

/*************************************************************************
	HandleStdDeviceReq
	==================
		Local function to handle a standard device request

	IN		pSetup		The setup packet
	IN/OUT	*piLen		Pointer to data length
			ppbData		Data buffer.

	Returns TRUE if the request was handled successfully
**************************************************************************/
static BOOL HandleStdDeviceReq(TSetupPacket *pSetup, int *piLen, U8 **ppbData)
{
	U8	*pbData = *ppbData;

	switch (pSetup->bRequest) {

	case REQ_GET_STATUS:
		// bit 0: self-powered
		// bit 1: remote wakeup
		pbData[0] = 0; 	// TODO use bmAttributes according to configuration
		pbData[1] = 0;
		*piLen = 2;
		break;

	case REQ_SET_ADDRESS:
		USBHwSetAddress(pSetup->wValue);
		break;

	case REQ_GET_DESCRIPTOR:
		DBG("D%x", pSetup->wValue);
		if (pfnGetDescriptor == NULL) {
			return FALSE;
		}
		return pfnGetDescriptor(pSetup->wValue, pSetup->wIndex, piLen, ppbData);

	case REQ_GET_CONFIGURATION:
		// indicate if we are configured
		pbData[0] = bConfiguration;
		*piLen = 1;
		break;

	case REQ_SET_CONFIGURATION:
		bConfiguration = pSetup->wValue & 0xFF;	// TODO use bConfigurationValue(s)
		USBHwConfigDevice((pSetup->wValue & 0xFF) != 0);
		break;

	case REQ_CLEAR_FEATURE:
	case REQ_SET_FEATURE:
		if (pSetup->wValue == FEA_REMOTE_WAKEUP) {
			// put DEVICE_REMOTE_WAKEUP code here
		}
		if (pSetup->wValue == FEA_TEST_MODE) {
			// put TEST_MODE code here
		}
		return FALSE;

	case REQ_SET_DESCRIPTOR:
		DBG("Device req %d not implemented\n", pSetup->bRequest);
		return FALSE;

	default:
		DBG("Illegal device req %d\n", pSetup->bRequest);
		return FALSE;
	}

	return TRUE;
}


/*************************************************************************
	HandleStdInterfaceReq
	=====================
		Local function to handle a standard interface request

	IN		pSetup		The setup packet
	IN/OUT	*piLen		Pointer to data length
			ppbData		Data buffer.

	Returns TRUE if the request was handled successfully
**************************************************************************/
static BOOL HandleStdInterfaceReq(TSetupPacket	*pSetup, int *piLen, U8 **ppbData)
{
	U8	*pbData = *ppbData;

	switch (pSetup->bRequest) {

	case REQ_GET_STATUS:
		// no bits specified
		pbData[0] = 0;
		pbData[1] = 0;
		*piLen = 2;
		break;

	case REQ_CLEAR_FEATURE:
	case REQ_SET_FEATURE:
		// not defined for interface
		return FALSE;

	case REQ_GET_INTERFACE:	// TODO use bNumInterfaces
        // there is only one interface, return n-1 (= 0)
		pbData[0] = 0;
		*piLen = 1;
		break;

	case REQ_SET_INTERFACE:	// TODO use bNumInterfaces
		// there is only one interface (= 0)
		if (pSetup->wValue == 0) {
			// ACK (zero packet) will be sent automatically
		}
		else {
			return FALSE;
		}
		break;

	default:
		DBG("Illegal interface req %d\n", pSetup->bRequest);
		return FALSE;
	}

	return TRUE;
}


/*************************************************************************
	HandleStdEndPointReq
	====================
		Local function to handle a standard endpoint request

	IN		pSetup		The setup packet
	IN/OUT	*piLen		Pointer to data length
			ppbData		Data buffer.

	Returns TRUE if the request was handled successfully
**************************************************************************/
static BOOL HandleStdEndPointReq(TSetupPacket	*pSetup, int *piLen, U8 **ppbData)
{
	U8	*pbData = *ppbData;

	switch (pSetup->bRequest) {
	case REQ_GET_STATUS:
		// bit 0 = endpointed halted or not
		pbData[0] = USBHwGetEPStall(pSetup->wIndex) ? 1 : 0;
		pbData[1] = 0;
		*piLen = 2;
		break;

	case REQ_CLEAR_FEATURE:
		if (pSetup->wValue == FEA_ENDPOINT_HALT) {
			// clear HALT by unstalling
			USBHwEPStall(pSetup->wIndex, FALSE);
			break;
		}
		// only ENDPOINT_HALT defined for endpoints
		return FALSE;

	case REQ_SET_FEATURE:
		if (pSetup->wValue == FEA_ENDPOINT_HALT) {
			// set HALT by stalling
			USBHwEPStall(pSetup->wIndex, TRUE);
			break;
		}
		// only ENDPOINT_HALT defined for endpoints
		return FALSE;

	case REQ_SYNCH_FRAME:
		DBG("EP req %d not implemented\n", pSetup->bRequest);
		return FALSE;

	default:
		DBG("Illegal EP req %d\n", pSetup->bRequest);
		return FALSE;
	}

	return TRUE;
}


/*************************************************************************
	USBHandleStandardRequest
	===================
		Local function to handle a standard request

	IN		pSetup		The setup packet
	IN/OUT	*piLen		Pointer to data length
			ppbData		Data buffer.

	Returns TRUE if the request was handled successfully
**************************************************************************/
BOOL USBHandleStandardRequest(TSetupPacket	*pSetup, int *piLen, U8 **ppbData)
{
	switch (REQTYPE_GET_RECIP(pSetup->bmRequestType)) {
	case REQTYPE_RECIP_DEVICE:		return HandleStdDeviceReq(pSetup, piLen, ppbData);
	case REQTYPE_RECIP_INTERFACE:	return HandleStdInterfaceReq(pSetup, piLen, ppbData);
	case REQTYPE_RECIP_ENDPOINT: 	return HandleStdEndPointReq(pSetup, piLen, ppbData);
	default: 						return FALSE;
	}
}


/*************************************************************************
	USBRegisterDescriptorHandler
	=========================
		Registers a callback for handling descriptors

	IN		pfnGetDesc	Callback function pointer

**************************************************************************/
void USBRegisterDescriptorHandler(TFnGetDescriptor *pfnGetDesc)
{
	pfnGetDescriptor = pfnGetDesc;
}


