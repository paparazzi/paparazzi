/*
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2002  Chris efstathiou hendrix@otenet.gr
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

/**
 * @file modules/sensors/alt_srf08.h
 * @brief Basic library for SRF08 telemeter
 *
 */


#ifndef SRF08_H
#define SRF08_H     1

/*###########################################################################*/
/*                      START OF CONFIGURATION BLOCK                         */
/*###########################################################################*/
/* Normally you shouldn't need to change anything */

#define SRF08_UNIT_0   0xE0  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_1   0xE2  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_2   0xE4  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_3   0xE6  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_4   0xE8  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_5   0xEA  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_6   0xEC  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_7   0xEE  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_8   0xF0  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_9   0xF2  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_10  0xF4  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_11  0xF6  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_12  0xF8  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_13  0xFA  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_14  0xFC  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_15  0xFE  /* the SRF08 MODULE I2C address */

/*###########################################################################*/
/*                    END OF CONFIGURATION BLOCK                             */
/*###########################################################################*/

#define SRF08_I2C_BROADCAST_ADDRESS   0X00

#define SRF08_MIN_GAIN        0      /* sets gain to 94   */
#define SRF08_MAX_GAIN        31     /* sets gain to 1025 */
#define SRF08_MIN_RANGE       0      /* in millimeters    */
#define SRF08_MAX_RANGE       11008  /* in millimeters    */

#define SRF08_INCHES          0X50
#define SRF08_CENTIMETERS     0X51
#define SRF08_MICROSECONDS    0X52

/* register positions */
#define SRF08_COMMAND         0
#define SRF08_SET_GAIN        1
#define SRF08_LIGHT           1
#define SRF08_ECHO_1          2
#define SRF08_ECHO_2          4
#define SRF08_ECHO_3          6
#define SRF08_ECHO_4          8
#define SRF08_ECHO_5          10
#define SRF08_ECHO_6          12
#define SRF08_ECHO_7          14
#define SRF08_ECHO_8          16
#define SRF08_ECHO_9          18
#define SRF08_ECHO_10         20
#define SRF08_ECHO_11         22
#define SRF08_ECHO_12         24
#define SRF08_ECHO_13         26
#define SRF08_ECHO_14         28
#define SRF08_ECHO_15         30
#define SRF08_ECHO_16         32
#define SRF08_ECHO_17         34

/* Function Declaration */

extern void srf08_select_unit(uint8_t srf08_address);
extern void srf08_init(void);

extern void srf08_set_gain(uint8_t gain);
extern void srf08_set_range(uint32_t millimeters);

extern void srf08_ping(void);
extern uint32_t srf08_read_register(uint8_t srf08_register);

extern void srf08_change_i2c_address(uint8_t new_i2c_address);

extern void srf08_initiate_ranging(void);
extern void srf08_receive(void);

extern uint16_t srf08_range;
extern bool_t srf08_received, srf08_got;
/** Read values on the bus */
extern void srf08_read(void);
/** Copy the I2C buffer */
extern void srf08_copy(void);

extern void srf08_event(void);

#endif  /* #ifndef SRF08_H */
