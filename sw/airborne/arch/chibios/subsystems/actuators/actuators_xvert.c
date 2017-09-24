/*
 * Copyright (C) Kevin van Hecke
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
 * @file actuators_xvert.c
 * @author Kevin van Hecke
 * Actuators driver for X-vert VTOL motor controllers. Contains two
 * normal pwm servos, and two custom driven escs through a propriety uart
 * protocol.
 */
#include "actuators_xvert.h"
#include "subsystems/actuators/actuators_pwm_arch.h"
#include "subsystems/actuators/actuators_pwm.h"

#include "mcu_periph/uart.h"

#define ESCS_PORT   (&((ESCS_UART).device))



int32_t actuators_xvert_values[ACTUATORS_PWM_NB];


#define GP  0x107   /* x^8 + x^2 + x + 1 */
#define DI  0xE7


static unsigned char crc8_table[256];     /* 8-bit table */
static int made_table=0;

static void init_crc8(void)
     /*
      * Should be called before any other crc function.
      */
{
  int i,j;
  unsigned char crc;

  if (!made_table) {
    for (i=0; i<256; i++) {
      crc = i;
      for (j=0; j<8; j++)
        crc = (crc << 1) ^ ((crc & 0x80) ? DI : 0);
      crc8_table[i] = crc & 0xFF;
      /* printf("table[%d] = %d (0x%X)\n", i, crc, crc); */
    }
    made_table=1;
  }
}


void crc8(unsigned char *crc, unsigned char m)
     /*
      * For a byte array whose accumulated crc value is stored in *crc, computes
      * resultant crc obtained by appending m to the byte array
      */
{
  if (!made_table)
    init_crc8();

  *crc = crc8_table[(*crc) ^ m];
  *crc &= 0xFF;
}


void actuators_xvert_init(void) {
    actuators_pwm_arch_init();
}



void actuators_xvert_commit(void) {
    if (ESCS_PORT->char_available(ESCS_PORT->periph)) {
        //unsigned char b1 = ESCS_PORT->get_byte(ESCS_PORT->periph);
    }

    struct EscData package;
    package.start = ESCS_START_BYTE;
    package.len = 8;
    package.id = 2;

    package.d1 =actuators_xvert_values[XVERT_ESC_0];
    package.d2 =actuators_xvert_values[XVERT_ESC_1];

    //do some package magic:
    static bool bitflipper = true;
    if (bitflipper)
        package.d1 += ESCS_DATA_FLIPBIT;
    else {
        package.d2 += ESCS_DATA_FLIPBIT;
    }
    package.d1 += ESCS_DATA_MYSTERYBIT;
    bitflipper = !bitflipper;

    unsigned char crc = 0;
    unsigned char * data = (unsigned char *)&package;
    for(unsigned char i = 1 ; i< sizeof(struct EscData)-1;i++) {
        crc8(&crc, data[i]);
    }
    package.crc = crc;

    ESCS_PORT->put_buffer(ESCS_PORT->periph, 0, (unsigned char *) &package, sizeof(struct EscData));


    //send the pwm signals for the two elerons to the pwm driver:
    actuators_pwm_values[PWM_SERVO_2] = actuators_xvert_values[PWM_SERVO_2];
    actuators_pwm_values[PWM_SERVO_3] = actuators_xvert_values[PWM_SERVO_3];
    actuators_pwm_commit();

}
