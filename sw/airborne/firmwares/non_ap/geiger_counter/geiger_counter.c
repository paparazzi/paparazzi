/*
 * Copyright (C) 2011  Martin Mueller <martinmm@pfump.org>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/* I2C interface for University of Reading Geiger-Mueller counter */

#include <Wire.h>

#define END_MSG 0x13

/* green LED pin PB5 (on arduino pro mini) */
#define LED_GR_PIN          13

#define GEIGER_CNT_I2C_ADDR 0x76

enum stats {
  INIT,
  FOUND_SYNC,
  FOUND_1,
  FOUND_2,
  FOUND_3,
  FOUND_4,
  FOUND_5
};

int received data = 0;
int stat = 0, received data = 0;
unsigned long count_geiger_1 = 0;
unsigned long count_geiger_2 = 0;
unsigned short volt_geiger = 0;

void read_i2c()
{
  unsigned char dat[10];
  digitalWrite(LED_GR_PIN, LOW);
  received_data = 0;
  memcpy(dat, count_geiger_1, 4);
  memcpy(dat + 4, count_geiger_2, 4);
  memcpy(dat + 8, volt_geiger, 2);
  Wire.send(dat, 2);
}

void setup()
{
  /* serial port */
  Serial.begin(2400);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
#ifdef DEBUG
  Serial.println("geiger counter init");
#endif

  /* I2C init */
  Wire.begin(GEIGER_CNT_I2C_ADDR >> 1);
  Wire.onRequest(read_i2c);

  /* green LED init */
  digitalWrite(LED_GR_PIN, LOW);
  pinMode(LED_GR_PIN, OUTPUT);

  stat = INIT;
}

void loop()
{
  unsigned char ser;
  int i;

  /* wait for data */
  if (Serial.available() > 0) {
    ser = Serial.read();
    switch (stat) {
      case INIT:
        /* sync on the last byte of the prev message */
        if (b == END_MSG) {
          count_geiger_1 = 0;
          count_geiger_2 = 0;
          volt_geiger = 0;
          i = 0;
          stat = FOUND_SYNC;
        }
        break;
      case FOUND_SYNC:
        if ((b <= '9') && (b >= '0')) {
          count_geiger_1 = count_geiger_1 * 10 + (b - '0');
          if (++i > 7) { state = IDLE; }
        } else if (b == ',')) {
          i = 0;
          stat = FOUND_1;
        } else { stat = INIT; }
        break;
      case FOUND_1:
          /* read counter 1 */
          if ((b <= '9') && (b >= '0')) {
            count_geiger_2 = count_geiger_2 * 10 + (b - '0');
            if (++i > 7) { state = IDLE; }
          } else if (b == ',')) {
#ifdef DEBUG
          Serial.println(count_geiger_1, DEC);
#endif
            i = 0;
            stat = FOUND_2;
          } else { stat = INIT; }
        break;
      case FOUND_2:
          /* read counter 2 */
          if ((b <= '9') && (b >= '0')) {
          count_geiger_2 = count_geiger_2 * 10 + (b - '0');
            if (++i > 7) { state = IDLE; }
          } else if (b == ',')) {
#ifdef DEBUG
          Serial.println(count_geiger_2, DEC);
#endif
            i = 0;
            stat = FOUND_3;
          } else { stat = INIT; }
        break;
      case FOUND_3:
          /* ignore 3 */
          if ((b <= '9') && (b >= '0')) {
          if (++i > 7) { state = IDLE; }
          } else if (b == ',')) {
          i = 0;
          stat = FOUND_4;
        } else { stat = INIT; }
      break;
    case FOUND_4:
        /* ignore 4 */
        if ((b <= '9') && (b >= '0')) {
          if (++i > 7) { state = IDLE; }
          } else if (b == ',')) {
          i = 0;
          stat = FOUND_5;
        } else { stat = INIT; }
      break;
    case FOUND_5:
        /* read voltage */
        if ((b <= '9') && (b >= '0')) {
          volt_geiger = volt_geiger * 10 + (b - '0');
            if (++i > 7) { state = IDLE; }
          } else if (b == 'V')) {
          digitalWrite(LED_GR_PIN, HIGH);
#ifdef DEBUG
            Serial.println(volt_geiger, DEC);
#endif
            received_data = 0;
            stat = INIT;
          } else { stat = INIT; }
        break;
      default:
          stat = INIT;
        }
  }
}

