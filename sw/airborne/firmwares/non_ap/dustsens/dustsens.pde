/*
 * $Id$
 *
 * Copyright (C) 2010 Martin Mueller, <martinmm@pfump.org>
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

/* I2C interface for Sharp GP2Y1010AU optical dust sensor */

#include <Wire.h>


/* infrared LED pin PB1, high active */
#define IR_FLASH_PIN    9
/* adc_in pin ADC0 */
#define ADC_PIN         0
/* green LED pin PB5 (on arduino pro mini) */
#define LED_GR_PIN      13

#define GP2Y_I2C_ADDR   0x76
#define AVG_NB_SAMPLE   25


struct adc_buf {
  unsigned long sum;
  unsigned short values[AVG_NB_SAMPLE];
  unsigned char head;
};

struct adc_buf buf;

unsigned char i2c_out[2];
unsigned long time_us;
unsigned long time_ms;
unsigned long trig_next;
unsigned long cnt;
unsigned short adc_val;
unsigned char new_head;


ISR(ADC_vect) {
  adc_val = ADCL;
  adc_val |= ADCH << 8;
  new_head = buf.head + 1;
  if (new_head >= AVG_NB_SAMPLE)
    new_head=0;
  buf.sum -= buf.values[new_head];
  buf.values[new_head] = adc_val;
  buf.sum += adc_val;
  buf.head = new_head;
}

void read_i2c() {
  i2c_out[0] = ((unsigned int)(buf.sum / AVG_NB_SAMPLE) >> 8) & 0xFF;
  i2c_out[1] = (unsigned int)(buf.sum / AVG_NB_SAMPLE) & 0xFF;

  Wire.send(i2c_out, 2);
}

void setup() { 
  /* I2C init */
  Wire.begin(GP2Y_I2C_ADDR);
  Wire.onRequest(read_i2c);

  /* green LED init */
  digitalWrite(LED_GR_PIN, LOW);
  pinMode(LED_GR_PIN, OUTPUT);
  
  /* infrared LED switch init */
  digitalWrite(IR_FLASH_PIN, HIGH);
  pinMode(IR_FLASH_PIN, OUTPUT);

  /* init adc, default reference */
  ADCSRA |= (1<<ADIE)|(1<<ADEN);
  ADCSRA |= (1<<ADSC);
  ADMUX = (DEFAULT << 6) | (ADC_PIN & 0x07);
  
  trig_next = millis() + 10;
  cnt = 0;
}

void loop() {
  if (millis() >= trig_next) {
    trig_next += 10;

    noInterrupts();

    time_us = micros();
    
    /* flash the infrared, LED on */
    digitalWrite(IR_FLASH_PIN, HIGH);

    /* delay 0.28ms for reflection */
    while((micros() - time_us) < 280);

    /* start conversion */
    ADCSRA |= 1 << ADSC;

    /* fill delay to 0.32ms for end of reflection */
    while((micros() - time_us) < 320);

    /* infrared LED off */
    digitalWrite(IR_FLASH_PIN, LOW);
    
    interrupts();

#if 0
    /* blink green LED */
    if (cnt++ > 90) {
      digitalWrite(LED_GR_PIN, HIGH);
      if (cnt > 100) {
        digitalWrite(LED_GR_PIN, LOW);
        cnt = 0;
      }
    }
#endif    
  }
}

