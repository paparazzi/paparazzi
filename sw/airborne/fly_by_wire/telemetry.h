/*
 * Paparazzi fbw mcu telemetry
 *  
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <inttypes.h>

#define STX 0x05
#define ETX 0x06

extern uint8_t ck_a, ck_b;
#include "uart.h"

void telemetry_send_fbw_status(uint8_t* nb_spi_err, uint8_t* rc_status, uint8_t* mode);

#define TELEMETRY_START_MESSAGE(id) {					\
    UART_PUT_1_BYTE(STX); UART_PUT_1_BYTE(id); ck_a = id; ck_b = id;	\
  }

#define TELEMETRY_END_MESSAGE() {					\
    UART_PUT_1_BYTE(ck_a); UART_PUT_1_BYTE(ck_b);			\
  }

#define TELEMETRY_CHECK_FREE_SPACE(_space) UART_CHECK_FREE_SPACE(_space)

#define TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR(_byte) { \
    UART_PUT_1_BYTE_BY_ADDR(_byte);		   \
    ck_a += *(_byte);				   \
    ck_b += ck_a;				   \
  }

#define TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(_byte) { \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR(_byte);	   \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR(_byte+1);	   \
  }

#define TELEMETRY_PUT_4_DATA_BYTE_BY_ADDR(_byte) { \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR(_byte);	   \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR(_byte+1);	   \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR(_byte+2);	   \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR(_byte+3);	   \
  }

#endif /* TELEMETRY_H */
