/*
 * $Id$
 *
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

/* I2C interface for University of Reading charge sensor */

#define DEBUG 1

#include <Wire.h>


/* in ms */
#define SAMPLE_RATE     100
/* x times samle rate */
#define RESET_RATE      50

/* x times over-sampling */
#define AVG_NB_SAMPLE   10

/* print every x values */
#define LED_RATE       10

/* charge reset pin PB1, high active */
#define RESET_PIN       9
/* adc_in pin ADC0 */
#define ADC_PIN         0
/* green LED pin PB5 (on arduino pro mini) */
#define LED_GR_PIN      13

#define CHARGE_SENS_I2C_ADDR   0x78
#define RESET_ACTIVE           0x1000

struct adc_buf {
  unsigned long sum;
  unsigned short values[AVG_NB_SAMPLE];
  unsigned char head;
};

struct adc_buf buf;

unsigned short reset_active_uart = 0;
unsigned short reset_active_i2c = 0;
unsigned long trig_next;
unsigned short adc_val;
unsigned char new_head;
int i, cnt = 0;
int i2c_cnt = 0;
int led_cnt = 0;
int reset_cnt = 0;


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
  unsigned short i2c;
  i2c = (buf.sum / AVG_NB_SAMPLE) | reset_active_i2c;
  reset_active_i2c = 0;

  Wire.send((uint8_t*)i2c, 2);
}

void setup() { 
#ifdef DEBUG
  /* serial port */
  Serial.begin(38400);
  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);
  Serial.println("charge sensor init");
#endif

  /* I2C init */
  Wire.begin(CHARGE_SENS_I2C_ADDR >> 1);
  Wire.onRequest(read_i2c);

  /* green LED init */
  digitalWrite(LED_GR_PIN, LOW);
  pinMode(LED_GR_PIN, OUTPUT);
  
  /* reset pin init */
  digitalWrite(RESET_PIN, LOW);
  pinMode(RESET_PIN, OUTPUT);

  /* init adc, default reference */
  ADCSRA |= (1<<ADIE)|(1<<ADEN);
  ADCSRA |= (1<<ADSC);
  ADMUX = (DEFAULT << 6) | (ADC_PIN & 0x07);
  
  trig_next = millis() + SAMPLE_RATE / AVG_NB_SAMPLE;
}

void loop() {
  if (millis() >= trig_next) {
    trig_next += SAMPLE_RATE / AVG_NB_SAMPLE;
    
    /* start conversion */
    ADCSRA |= 1 << ADSC;
    
    if (++cnt >= AVG_NB_SAMPLE) {
      cnt = 0;
      
      if (++reset_cnt >= RESET_RATE) {
        /* start reset */
        digitalWrite(RESET_PIN, HIGH);
        reset_cnt = 0;
        reset_active_uart = RESET_ACTIVE;    
        reset_active_i2c = RESET_ACTIVE;    
      }
      else {
        /* stop reset */
        digitalWrite(RESET_PIN, LOW);
      }

#ifdef DEBUG
      Serial.print((buf.sum / AVG_NB_SAMPLE) | reset_active_uart);
      Serial.print(" ");
#endif
      if (++led_cnt >= LED_RATE) {
        led_cnt = 0;
        /* blink green LED */
        digitalWrite(LED_GR_PIN, HIGH);
#ifdef DEBUG
        Serial.println("");
#endif
      } 
      else digitalWrite(LED_GR_PIN, LOW);
      reset_active_uart = 0;           
    }
  }
}

