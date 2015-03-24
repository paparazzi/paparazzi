/*
 * Copyright (C) 2011 ENAC - Gautier Hattenberger
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
 */

/**
 * @file subsystems/abi_common.h
 *
 * Common tools for ABI middelware.
 */

#ifndef ABI_COMMON_H
#define ABI_COMMON_H

/* Include pprz math library */
#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "subsystems/gps.h"
/* Include here headers with structure definition you may want to use with ABI
 * Ex: '#include "subsystems/gps.h"' in order to use the GpsState structure
 */

#include "subsystems/abi_sender_ids.h"

/* Some magic to avoid to compile C code, only headers */
#ifdef ABI_C
#define ABI_EXTERN
#else
#define ABI_EXTERN extern
#endif

/** Generic callback definition */
typedef void (*abi_callback)(void);

/** Broadcast address.
 * When passing broadcast address as a sender id,
 * messages from all senders are received
 */
#define ABI_BROADCAST 0

/** Event structure to store callbacks in a linked list */
struct abi_struct {
  uint8_t id;
  abi_callback cb;
  struct abi_struct *next;
};
typedef struct abi_struct abi_event;

/** Macros for linked list */
#define ABI_FOREACH(head,el) for(el=head; el; el=el->next)
#define ABI_PREPEND(head,add) { (add)->next = head; head = add; }

#endif /* ABI_COMMON_H */

