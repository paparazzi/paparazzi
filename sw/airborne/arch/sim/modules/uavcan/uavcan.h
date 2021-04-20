/*
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file arch/sim/modules/uavcan/uavcan.h
 * Stub interface for simulation
 */
#ifndef MODULES_UAVCAN_ARCH_H
#define MODULES_UAVCAN_ARCH_H

#include <std.h>
#include <string.h>

/** uavcan interface structure */
struct uavcan_iface_t {};

/** Generic uavcan callback definition */
struct CanardRxTransfer_t {};
typedef struct CanardRxTransfer_t CanardRxTransfer;
typedef void (*uavcan_callback)(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);

/** Main uavcan event structure for registering/calling callbacks */ 
struct uavcan_event_t {};
typedef struct uavcan_event_t uavcan_event;

/** uavcan interfaces */
#if UAVCAN_USE_CAN1
extern struct uavcan_iface_t uavcan1;
#endif
#if UAVCAN_USE_CAN2
extern struct uavcan_iface_t uavcan2;
#endif

/** uavcan external functions */
void uavcan_init(void);
void uavcan_bind(uint16_t data_type_id, uint64_t data_type_signature, uavcan_event *ev, uavcan_callback cb);
void uavcan_broadcast(struct uavcan_iface_t *iface, uint64_t data_type_signature, uint16_t data_type_id, uint8_t priority,
                      const void* payload, uint16_t payload_len);

#endif /* MODULES_UAVCAN_ARCH_H */