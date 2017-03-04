/*
 * Copyright (C) 2017 The Paparazzi Team
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
 * @file modemhumid.ino
 *
 * modem and humidity sensor for Parrot Bebop2 using two serial ports
 * through a Sparkfun FIO v3 with XBee and SHT75 sensor
 *
 * requires DUALCDCUSB library, tested with Arduino v1.8.1
 *
 */

#include <DUALCDCUSB.h>

int RXLED = 17;

void setup()
{
  pinMode(RXLED, OUTPUT);  // Set RX LED as an output

  Serial.begin(57600);  // USB CDC 0 port
  Serial1.begin(57600); // serial port
  Serial2.begin(57600); // USB CDC 1 port

  TXLED1;    // set the LED off
  digitalWrite(RXLED, HIGH);    // set the LED off

  humid_sht_init();
}

void loop()
{
  int i, av, ret;
  size_t re;
  unsigned char buff[256], device_addr = 0xF0, len = 2;
  unsigned short tmd_temperature;
  float ftmd_temperature;
  static unsigned long ticks = 0, last_ticks = 0;

 while (1) {

  /* USB CDC 0 -> UART */
  av = Serial.available();
  if (av > 0) {
    if (av > 256)
      av = 256;
    ret = Serial.readBytes(buff, av);
    Serial1.write(buff, ret);
  }

  /* UART -> USB CDC 0*/
  av = Serial1.available();
  if (av > 0) {
    if (av > 256)
      av = 256;
    ret = Serial1.readBytes(buff, av);
    Serial.write(buff, ret);
  }  

  ticks = millis();
  if (last_ticks == 0) last_ticks = ticks;
  if (last_ticks + 2 < ticks) {
    last_ticks = ticks;
    humid_sht_periodic();
  }
 }
}
