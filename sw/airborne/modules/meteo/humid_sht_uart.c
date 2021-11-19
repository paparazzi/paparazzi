/*
 * Copyright (C) 2008-2017 The Paparazzi team
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
 * @file modules/meteo/humid_sht_uart.c
 *
 * SHTxx sensor interface
 *
 * This reads the values for humidity and temperature from the SHTxx sensor through an uart.
 *
 */

#include "std.h"
#include "mcu_periph/gpio.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "humid_sht_uart.h"

uint16_t humidsht, tempsht;
float fhumidsht, ftempsht;
bool humid_sht_available;

void calc_sht(uint16_t hum, uint16_t tem, float *fhum , float *ftem);
void humid_sht_uart_parse(uint8_t c);

void calc_sht(uint16_t hum, uint16_t tem, float *fhum , float *ftem)
{
  // calculates temperature [ C] and humidity [%RH]
  // input : humi [Ticks] (12 bit)
  //             temp [Ticks] (14 bit)
  // output: humi [%RH]
  //             temp [ C]

  const float C1 = -4.0;            // for 12 Bit
  const float C2 = 0.0405;          // for 12 Bit
  const float C3 = -0.0000028;      // for 12 Bit
  const float T1 = 0.01;            // for 14 Bit @ 5V
  const float T2 = 0.00008;         // for 14 Bit @ 5V
  float rh;                         // rh:      Humidity [Ticks] 12 Bit
  float t;                          // t:       Temperature [Ticks] 14 Bit
  float rh_lin;                     // rh_lin:  Humidity linear
  float rh_true;                    // rh_true: Temperature compensated humidity
  float t_C;                        // t_C   :  Temperature [ C]

  rh = (float)hum;                  //converts integer to float
  t = (float)tem;                   //converts integer to float

  t_C = t * 0.01 - 39.66;           //calc. Temperature from ticks to [°C] @ 3.5V
  rh_lin = C3 * rh * rh + C2 * rh + C1; //calc. Humidity from ticks to [%RH]
  rh_true = (t_C - 25) * (T1 + T2 * rh) + rh_lin; //calc. Temperature compensated humidity [%RH]
  if (rh_true > 100) { rh_true = 100; } //cut if the value is outside of
  if (rh_true < 0.1) { rh_true = 0.1; } //the physical possible range
  *ftem = t_C;                      //return temperature [ C]
  *fhum = rh_true;                  //return humidity[%RH]
}

void humid_sht_uart_periodic(void)
{
}

/* airspeed_otf_parse */
void humid_sht_uart_parse(uint8_t c)
{
  static uint8_t msg_cnt = 0;
  static uint8_t data[6];
  uint16_t i, chk = 0;

  if (msg_cnt > 0) {
    data[msg_cnt++] = c;
    if (msg_cnt == 6) {
      tempsht = data[1] | (data[2] << 8);
      humidsht = data[3] | (data[4] << 8);
      for (i = 1; i < 5; i++)
        chk += data[i];
      if (data[5] == (chk & 0xFF)) {
        calc_sht(humidsht, tempsht, &fhumidsht, &ftempsht);
        DOWNLINK_SEND_SHT_STATUS(DefaultChannel, DefaultDevice, &humidsht, &tempsht, &fhumidsht, &ftempsht);
	  }
      msg_cnt = 0;
    }
  }
  else if (c == 0xFF)
    msg_cnt = 1;
}

void humid_sht_uart_init(void)
{
}

void humid_sht_uart_event(void)
{
  while (MetBuffer()) {
    uint8_t ch = MetGetch();
    humid_sht_uart_parse(ch);
  }
}
