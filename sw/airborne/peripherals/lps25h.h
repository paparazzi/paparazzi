/*
 * Copyright (C) 2019 Alexis Cornard <alexiscornard@gmail.com>
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
 * @file peripherals/lps25h.h
 *
 * LPS25H barometer driver interface.
 */

#ifndef LPS25H_H
#define LPS25H_H


#include "std.h"

/* Include address and register definition */
#include "peripherals/lps25h_regs.h"

enum Lps25hConfStatus {
    LPS25H_CONF_UNINIT,
    LPS25H_CONF_CTRL1,
    LPS25H_CONF_DONE
};

struct Lps25hConfig {
    uint8_t ctrl1;
};

static inline void lps25h_set_default_config(struct Lps25hConfig *c)
{
  c->ctrl1 = 0xB0;
}


#endif // LPS25H_H
