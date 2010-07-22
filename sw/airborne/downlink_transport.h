/*
 * Paparazzi $Id: downlink_transport.h 4766 2010-03-30 10:00:29Z hecto $
 *  
 * Copyright (C) 2003-2006  Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

/** \file downlink_transport.h
 *  \brief Interface for downlink transport implementation
 *
 */

#ifndef DOWNLINK_TRANSPORT_H
#define DOWNLINK_TRANSPORT_H

#include <inttypes.h>

struct DownlinkTransport
{
	uint8_t (*SizeOf)(void *impl, uint8_t size);
	int (*CheckFreeSpace)(void *impl, uint8_t size);

	void (*PutBytes)(void *impl, uint8_t len, const uint8_t *bytes);

	void (*StartMessage)(void *impl, char *name, uint8_t msg_id, uint8_t payload_len);
	void (*EndMessage)(void *impl);
	void (*Overrun)(void *impl);
	void (*CountBytes)(void *impl, uint8_t len);

	void *impl;
};

#endif /* DOWNLINK_TRANSPORT_H */
