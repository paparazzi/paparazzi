/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file arch/linux/mcu_periph/udp_arch.h
 * linux UDP handling
 */

#ifndef UDP_ARCH_H
#define UDP_ARCH_H

#include "mcu_periph/udp.h"
#include "udp_socket.h"

extern void udp_arch_init(void);

/**
 * GCS computers found by the UDP links with host "auto" (see udp_arch.c), e.g. to send video to them too.
 * @param[out] addr  IP addresses of the GCS computers (each address once)
 * @param[in]  max   size of addr
 * @return number of addresses (0: no GCS found, or no link with host "auto")
 */
extern int udp_arch_auto_gcs(struct in_addr *addr, int max);

#endif /* UDP_ARCH_H */
