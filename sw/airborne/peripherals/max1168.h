/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 Gautier Hattenberger
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

#ifndef MAX1168_H
#define MAX1168_H

#include "std.h"

#define MAX1168_NB_CHAN 8

#ifndef MAX1168_SLAVE_IDX
#define MAX1168_SLAVE_IDX SPI_SLAVE0
#endif

#ifndef MAX1168_SPI_DEV
#define MAX1168_SPI_DEV spi1
#endif

/** Configure control register.
 */
#ifndef MAX1168_CONF_CHANNEL
#define MAX1168_CONF_CHANNEL 0x7 // select all channels
#endif
#ifndef MAX1168_CONF_SCAN
#define MAX1168_CONF_SCAN 0x1 // scan number of channels selected
#endif
#ifndef MAX1168_CONF_REF
#define MAX1168_CONF_REF 0x0 // internal ref and no power down
#endif
#ifndef MAX1168_CONF_CLOCK
#define MAX1168_CONF_CLOCK 0x1 // internal clock
#endif

#define MAX1168_CONF_CR ((MAX1168_CONF_CHANNEL<<5)|(MAX1168_CONF_SCAN<<3)|(MAX1168_CONF_REF<<1)|(MAX1168_CONF_CLOCK))

extern void max1168_init(void);
extern void max1168_read(void);
extern void max1168_event(void);

#define MAX1168_IDLE            0
#define MAX1168_SENDING_REQ     1
#define MAX1168_GOT_EOC         2
#define MAX1168_READING_RES     3
#define MAX1168_DATA_AVAILABLE  4
extern volatile uint8_t max1168_status;

extern uint16_t max1168_values[MAX1168_NB_CHAN];

#define Max1168Periodic() {             \
    if (max1168_status == MAX1168_IDLE) { \
      max1168_read();                     \
    }                                     \
  }

/* underlying architecture */
#include "peripherals/max1168_arch.h"
/* must be implemented by underlying architecture */
extern void max1168_arch_init(void);


#endif /* MAX1168_H */
