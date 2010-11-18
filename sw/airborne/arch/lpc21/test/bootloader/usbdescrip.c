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
	Default descriptor handler.
*/


#include "type.h"
#include "usbdebug.h"

#include "usbapi.h"

static const U8	*pabDescrip = NULL;


/*************************************************************************
	USBRegisterDescriptors
	======================
		Registers a pointer to a descriptor block containing all descriptors
		for the device.

	IN		pabDescriptors	The descriptor byte array

**************************************************************************/
void USBRegisterDescriptors(const U8 *pabDescriptors)
{
	pabDescrip = pabDescriptors;
}


/*************************************************************************
	USBHandleDescriptor
	===================
		Parses a previously installed descriptor block and attempts to find
		the specified USB descriptor.

	IN		wTypeIndex	Type and index of the descriptor
			wLangID		Language ID of the descriptor (currently unused)
	OUT		*piLen		Descriptor length
			*ppbData	Descriptor data

**************************************************************************/
BOOL USBHandleDescriptor(U16 wTypeIndex, U16 wLangID, int *piLen, U8 **ppbData)
{
	U8	bType, bIndex;
	U8	*pab;
	int iCurIndex;

	ASSERT(pabDescrip != NULL);

	bType = GET_DESC_TYPE(wTypeIndex);
	bIndex = GET_DESC_INDEX(wTypeIndex);

	pab = (U8 *)pabDescrip;
	iCurIndex = 0;

	while (pab[0] != 0) {
		if (pab[1] == bType) {
			if (iCurIndex == bIndex) {
				// set data pointer
				*ppbData = pab;
				// get length from structure
				if (bType == DESC_CONFIGURATION) {
					// interface descriptor is an exception, length is at offset 2 and 3
					*piLen = (pab[3] << 8) | pab[2];
				}
				else {
					// normally length is at offset 0
					*piLen = pab[0];
				}
				return TRUE;
			}
			iCurIndex++;
		}
		// skip to next descriptor
		pab += pab[0];
	}
	// nothing found
	DBG("Desc %x not found!\n", wTypeIndex);
	return FALSE;
}


