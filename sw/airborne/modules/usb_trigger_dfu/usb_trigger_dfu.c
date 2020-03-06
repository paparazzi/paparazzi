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
 * @file "modules/usb_trigger_dfu/usb_trigger_dfu.c"
 * @author Tom van Dijk
 * Use USB voltage as trigger to reboot to DFU mode.
 */

#include "modules/usb_trigger_dfu/usb_trigger_dfu.h"

#include "arch/stm32/mcu_arch.h"

void usb_trigger_dfu_init(void) {
  // Enable USB VBUS pin
  // May conflict with other USB code, but they should not be combined anyway
  // for obvious reasons (i.e. resetting when USB is plugged in).
}

void usb_trigger_dfu_periodic(void) {
  // XXX test, always reset to dfu
  reset_to_dfu();
}


