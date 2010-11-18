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

#include "type.h"
#include "usbdebug.h"
#include "usbapi.h"


/*************************************************************************
	HandleUsbReset
	==============
**************************************************************************/
static void HandleUsbReset(U8 bDevStatus)
{
	if (bDevStatus & DEV_STATUS_RESET) {
		USBHwReset();
		DBG("\n!");
	}
}


/*************************************************************************
	USBInit
	=======
		Initialises the USB hardware and sets up the USB stack by
		installing default callbacks.

**************************************************************************/
BOOL USBInit(void)
{
	// init hardware
	USBHwInit();

	// register bus reset handler
	USBHwRegisterDevIntHandler(HandleUsbReset);

	// register control transfer handler on EP0
	USBHwRegisterEPIntHandler(0x00, MAX_PACKET_SIZE0, USBHandleControlTransfer);
	USBHwRegisterEPIntHandler(0x80, MAX_PACKET_SIZE0, USBHandleControlTransfer);

	// register standard request handler
	USBRegisterRequestHandler(REQTYPE_TYPE_STANDARD, USBHandleStandardRequest);

	// register
	USBRegisterDescriptorHandler(USBHandleDescriptor);

	return TRUE;
}

