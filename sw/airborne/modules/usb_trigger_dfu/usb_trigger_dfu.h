/*
 * Copyright (C) Tom van Dijk
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/usb_trigger_dfu/usb_trigger_dfu.h"
 * @author Tom van Dijk
 * Use USB voltage as trigger to reboot to DFU mode.
 */

#ifndef USB_TRIGGER_DFU_H
#define USB_TRIGGER_DFU_H

void usb_trigger_dfu_init(void);
void usb_trigger_dfu_periodic(void);

#endif

