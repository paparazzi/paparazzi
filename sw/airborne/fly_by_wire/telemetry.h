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
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR((uint8_t*)_byte+1);	   \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR((uint8_t*)_byte);	   \
  }

#define TELEMETRY_PUT_4_DATA_BYTE_BY_ADDR(_byte) {                 \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR((uint8_t*)_byte+3);	   \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR((uint8_t*)_byte+2);	   \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR((uint8_t*)_byte+1);	   \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR((uint8_t*)_byte);	           \
  }

#define DL_PPM        0x01
#define DL_RC         0x02
#define DL_SERVOS     0x03
#define DL_FBW_STATUS 0x04

#define TELEMETRY_SEND_PPM(nb_channels, sync_len, ppm01, ppm02, ppm03,ppm04, ppm05, ppm06, ppm07, ppm08, ppm09, ppm10, ppm11, ppm12) { \
  if (TELEMETRY_CHECK_FREE_SPACE(30)) { \
    TELEMETRY_START_MESSAGE(DL_PPM); \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR(nb_channels); \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR(sync_len); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ppm01); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ppm02); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ppm03); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ppm04); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ppm05); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ppm06); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ppm07); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ppm08); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ppm09); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ppm10); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ppm11); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ppm12); \
    TELEMETRY_END_MESSAGE();		      \
  }					      \
}

#define TELEMETRY_SEND_RC(nb_channels, pprz01, pprz02, pprz03, pprz04, pprz05, pprz06, pprz07, pprz08, pprz09, pprz10, pprz11, pprz12) { \
  if (TELEMETRY_CHECK_FREE_SPACE(29)) { \
    TELEMETRY_START_MESSAGE(DL_RC); \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR(nb_channels); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(pprz01); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(pprz02); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(pprz03); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(pprz04); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(pprz05); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(pprz06); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(pprz07); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(pprz08); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(pprz09); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(pprz10); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(pprz11); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(pprz12); \
    TELEMETRY_END_MESSAGE(); \
  } \
}

#define TELEMETRY_SEND_SERVOS(nb_servos, ser01, ser02, ser03, ser04, ser05, ser06, ser07, ser08, ser09, ser10) { \
  if (TELEMETRY_CHECK_FREE_SPACE(25)) { \
    TELEMETRY_START_MESSAGE(DL_SERVOS); \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR(nb_servos); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ser01); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ser02); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ser03); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ser04); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ser05); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ser06); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ser07); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ser08); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ser09); \
    TELEMETRY_PUT_2_DATA_BYTE_BY_ADDR(ser10); \
    TELEMETRY_END_MESSAGE(); \
  } \
}


#define TELEMETRY_SEND_FBW_STATUS(nb_spi_err, rc_status, mode) { \
  if (TELEMETRY_CHECK_FREE_SPACE(7)) { \
    TELEMETRY_START_MESSAGE(DL_FBW_STATUS); \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR(nb_spi_err); \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR(rc_status); \
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR(mode); \
    TELEMETRY_END_MESSAGE(); \
  } \
}


#endif /* TELEMETRY_H */
