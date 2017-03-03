/*
 * Copyright (C) 2013 Alexandre Bustico, Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file subsystems/radio_control/hott_common.c
 *
 * Futaba HOTT decoder
 */

#include "subsystems/radio_control.h"
#include "subsystems/radio_control/hott_common.h"
#include BOARD_CONFIG
#include <string.h>

/*
 * HOTT protocol and state machine status
 */
#define HOTT_START_BYTE 0xa8

#define HOTT_STATUS_UNINIT      0
#define HOTT_STATUS_GOT_START   1
#define HOTT_STATUS_GOT_HEADER1   2
#define HOTT_STATUS_DATA   3

/** Set polarity using RC_POLARITY_GPIO.
 * HOTT signal has a reversed polarity compared to normal UART
 * this allows to using hardware UART peripheral by changing
 * the input signal polarity.
 * Setting this gpio ouput high inverts the signal,
 * output low sets it to normal polarity.
 */
#ifndef RC_SET_POLARITY
#define RC_SET_POLARITY gpio_set
#endif


void hott_common_init(struct SHott *hott_p, struct uart_periph *dev,
                      gpio_port_t gpio_polarity_port, uint16_t gpio_polarity_pin)
{
  hott_p->frame_available = false;
  hott_p->status = HOTT_STATUS_UNINIT;

  // Set UART parameters (115200, 8 bits, 1 stops, UPARITY_NONE)
  uart_periph_set_baudrate(dev, B115200);
  uart_periph_set_bits_stop_parity(dev, UBITS_8, USTOP_1, UPARITY_NO);
  // Try to invert RX data logic when available in hardware periph
//  uart_periph_invert_data_logic(dev, true, false);

  // Set polarity (when not done in hardware, don't use both!)
//  if (gpio_polarity_port != 0) {
//    gpio_setup_output(gpio_polarity_port, gpio_polarity_pin);
//    RC_SET_POLARITY(gpio_polarity_port, gpio_polarity_pin);
//  }

}

#define HOTT_CRC_POLYNOME 0x1021
/******************************************
************************************* 
* Function Name : CRC16 
* Description : crc calculation, adds a 8 bit unsigned to 16 bit crc 
*******************************************************************************/ 
static uint16_t hott_CRC16(uint16_t crc, uint8_t value) 
{
    uint8_t i; 
    crc = crc ^ (int16_t)value<<8; 
    for(i=0; i<8; i++) {
        if (crc & 0x8000)
            crc = (crc << 1) ^ HOTT_CRC_POLYNOME;
        else
            crc = (crc << 1); 
    }
    return crc; 
}

/** Decode the raw buffer */
static void decode_hott_buffer(const uint8_t *src, uint16_t *dst, uint8_t channels, bool *available,
                               uint16_t *dstppm __attribute__((unused)))
{
  // decode hott data
    uint8_t i;
    uint16_t sumd_crc_rx, sumd_crc=0;
    sumd_crc = hott_CRC16(sumd_crc, src[0]);
    sumd_crc = hott_CRC16(sumd_crc, src[1]);
    sumd_crc = hott_CRC16(sumd_crc, src[2]);
    for(i=0;i<channels;i++)
    {
        sumd_crc = hott_CRC16(sumd_crc, src[i*2+3]);
        sumd_crc = hott_CRC16(sumd_crc, src[i*2+4]);
        dst[i]  = (uint16_t)src[i*2 + 3]<<8 | (uint16_t)src[i*2+4];
    }
    sumd_crc_rx = (uint16_t)src[channels*2 +3]<<8 | (uint16_t)src[channels*2+4]; 
    
  // convert hott to ppm
#if PERIODIC_TELEMETRY
  for (int channel = 0; channel < HOTT_NB_CHANNEL; channel++) {
    dstppm[channel] = USEC_OF_RC_PPM_TICKS(dst[channel]);
  }
#endif

  // test crc
  *available = sumd_crc == sumd_crc_rx;
}


// Decoding event function
// Reading from UART
void hott_common_decode_event(struct SHott *hott_p, struct uart_periph *dev)
{
  uint8_t rbyte;
  if (uart_char_available(dev)) {
    do {
      rbyte = uart_getch(dev);
      switch (hott_p->status) {
        case HOTT_STATUS_UNINIT:
          // Wait for the start byte
          hott_p->idx = 0;
          if (rbyte == HOTT_START_BYTE) {
            hott_p->status++;
            hott_p->expected_channels = 0;
            hott_p->buffer[hott_p->idx] = rbyte;
            hott_p->idx++;
          }
          break;
        case HOTT_STATUS_GOT_START:
          if (rbyte ==0x01 || rbyte ==0x81) { // hott status
              /*
               * SUMD_Header Byte 1 Status 
               * 0x01 valid and live SUMD data frame
               * 0x81 valid SUMD data frame with transmitter in fail safe condition. 
               * Note:
               * The SUMD_Data section contains 
               * valid channel data. The channel data are 
               * set by transmitter fail safe values. A FBL 
               * system may replace the transmitter fail safe 
               * data by FBL stored values
               * 
               * other values Values different to 0x01 or 0x81 indicate an invalid SUMD data frame and 
               * should not be  processed by SUMD algorithms
               * */
            hott_p->buffer[hott_p->idx] = rbyte;
            hott_p->idx++;
            hott_p->status++;
          } else 
          {
              hott_p->status=0; // reset
          }
          break;
        case HOTT_STATUS_GOT_HEADER1:
            /*Indicates the number of channels transmitted in the SUMD_Data section
             */
          if (rbyte >=2 && rbyte <= HOTT_NB_CHANNEL) {
            hott_p->buffer[hott_p->idx] = rbyte;
            hott_p->idx++;
            hott_p->expected_channels = rbyte;
            hott_p->status++;
          }else {
              hott_p->status=0; // reset
          }
          break;
        case HOTT_STATUS_DATA:
          // Store buffer
            hott_p->buffer[hott_p->idx] = rbyte;
            hott_p->idx++;
            // Decode if last byte is (one of) the correct end byte
            if(hott_p->idx >= (hott_p->expected_channels*2 +2+3)) // 3 bytes header, 2 bytes crc
            {
                decode_hott_buffer(hott_p->buffer, hott_p->pulses , hott_p->expected_channels, &hott_p->frame_available, hott_p->ppm);
                hott_p->status = HOTT_STATUS_UNINIT;
            }
          break;
        default:
          break;
      }
    } while (uart_char_available(dev));
  }
}
