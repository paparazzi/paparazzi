/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *               2010 Eric Parsonage <eric@eparsonage.com>
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/**
 * @file modules/radio_control/spektrum.c
 *
 * Spektrum sattelite receiver implementation. For the protocol specification see:
 * http://www.spektrumrc.com/ProdInfo/Files/Remote%20Receiver%20Interfacing%20Rev%20A.pdf
 */

#include "std.h"
#include "modules/radio_control/spektrum.h"
#include "modules/radio_control/radio_control.h"
#include "modules/core/abi.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/gpio.h"
#include "mcu_periph/sys_time.h"

#if RADIO_CONTROL_NB_CHANNEL < SPEKTRUM_NB_CHANNEL
#warning "RADIO_CONTROL_NB_CHANNEL mustn't be lower than 14. X-Plus channel expansion is not (yet) usable"
#endif

/* Check if primary receiver is defined */
#ifndef SPEKTRUM_PRIMARY_UART
#error "You must at least define the primary Spektrum satellite receiver."
#endif

/* Number of low pulses sent during binding to the satellite receivers
 * Spektrum documentation recommend that master and slave receivers
 * should be configured in DSMX 11ms mode, other modes (DSM2, 22ms) will be
 * automatically supported if transmitter is not compatible.
 * But this this is only if receiver can handle DSMX. If it is not the case,
 * DSM2 must be used, otherwise the system filed will be wrong.
 * So DSM2 is used by default and DSMX can be enable with USE_DSMX flag.
 */
#if USE_DSMX
#define SPEKTRUM_MASTER_RECEIVER_PULSES 9 // only one receiver should be in master mode
#define SPEKTRUM_SLAVE_RECEIVER_PULSES 10
#else
#define SPEKTRUM_MASTER_RECEIVER_PULSES 5 // only one receiver should be in master mode
#define SPEKTRUM_SLAVE_RECEIVER_PULSES 6
#endif

/* Set polarity using RC_POLARITY_GPIO. */
#ifndef RC_SET_POLARITY
#define RC_SET_POLARITY gpio_clear
#endif

/* Busy wait to let the receiver starts properly
 * This should be reduced when the MCU takes longer to start
 */
#ifndef SPEKTRUM_BIND_WAIT
#define SPEKTRUM_BIND_WAIT 60000
#endif

/* Spektrum system type can be force (see list below)
 * by default it is unknown type and will be determined from incoming frames
 */
#ifndef SPEKTRUM_SYS_TYPE
#define SPEKTRUM_SYS_TYPE 0 // unknown type, determined from incoming frame
#endif

// in case the number of channel is less than maximum
const int8_t spektrum_signs[] = RADIO_CONTROL_SPEKTRUM_SIGNS;

/* Default spektrum values */
static struct spektrum_t spektrum = {
  .valid = false,
  .tx_type = SPEKTRUM_SYS_TYPE, // unknown type by default
};

/** Allowed system field valaues.
 * from datasheet, possible values for the frame format
 * encoded in the system field of the internal remote
 * frame (second byte).
 * Only the first one is encoding values on 10bits,
 * other ones on 11bits.
 */
#define SPEKTRUM_SYS_22_1024_2 0x01 // 22ms 1024 DSM2
#define SPEKTRUM_SYS_11_2048_2 0x12 // 11ms 2048 DSM2
#define SPEKTRUM_SYS_22_2048_X 0xa2 // 22ms 2048 DSMX
#define SPEKTRUM_SYS_11_2048_X 0xb2 // 11ms 2048 DSMX

static void spektrum_bind(void);

/** Initialize a spektrum sattelite */
static inline void spektrum_init_sat(struct spektrum_sat_t *sat)
{
  sat->valid = false;
  sat->timer = get_sys_time_msec();
  sat->idx = 0;

  // Initialize values
  for (uint8_t i = 0; i < SPEKTRUM_MAX_CHANNELS; i++) {
    sat->values[i] = 0;
  }
}

/*****************************************************************************
 *
 * spektrum_try_bind(void) must called on powerup as spektrum
 * satellites can only bind immediately after power up also it must be called
 * before the call to SpektrumUartInit as we leave them with their Rx pins set
 * as outputs.
 *
 *****************************************************************************/
void spektrum_try_bind(void)
{
#ifdef SPEKTRUM_BIND_PIN_PORT
#ifdef SPEKTRUM_BIND_PIN_HIGH
  /* Init GPIO for the bind pin, we enable the pulldown resistor.
   * (esden) As far as I can tell only navstick is using the PIN LOW version of
   * the bind pin, but I assume this should not harm anything. If I am mistaken
   * than I appologise for the inconvenience. :)
   */
  gpio_setup_input_pulldown(SPEKTRUM_BIND_PIN_PORT, SPEKTRUM_BIND_PIN);

  sys_time_usleep(10); // wait for electrical level to stabilize

  /* Exit if the BIND_PIN is low, it needs to
     be pulled high at startup to initiate bind */
  if (gpio_get(SPEKTRUM_BIND_PIN_PORT, SPEKTRUM_BIND_PIN) != 0) {
    spektrum_bind();
  }
#else
  /* Init GPIO for the bind pin, we enable the pullup resistor in case we have
   * a floating pin that does not have a hardware pullup resistor as it is the
   * case with Lisa/M and Lisa/MX prior to version 2.1.
   */
  gpio_setup_input_pullup(SPEKTRUM_BIND_PIN_PORT, SPEKTRUM_BIND_PIN);

  sys_time_usleep(10); // wait for electrical level to stabilize

  /* Exit if the BIND_PIN is high, it needs to
     be pulled low at startup to initiate bind */
  if (gpio_get(SPEKTRUM_BIND_PIN_PORT, SPEKTRUM_BIND_PIN) == 0) {
    spektrum_bind();
  }
#endif
#endif
}


/** Main Radio initialization */
void spektrum_init(void)
{

  for (uint8_t i = 0; i < SPEKTRUM_NB_CHANNEL; i++) {
    spektrum.signs[i] = spektrum_signs[i];
  }
  radio_control.nb_channel = SPEKTRUM_NB_CHANNEL;

  // Set polarity to normal on boards that can change this
#ifdef RC_POLARITY_GPIO_PORT
  gpio_setup_output(RC_POLARITY_GPIO_PORT, RC_POLARITY_GPIO_PIN);
  RC_SET_POLARITY(RC_POLARITY_GPIO_PORT, RC_POLARITY_GPIO_PIN);
#endif

  // Initialize all the UART's in the satellites
  spektrum_init_sat(&spektrum.satellites[0]);
#ifdef SPEKTRUM_SECONDARY_UART
  spektrum_init_sat(&spektrum.satellites[1]);
#endif
}

/* Parse a sattelite channel */
static inline void spektrum_parse_channel(struct spektrum_sat_t *sat, uint16_t chan)
{
  // This channel is not used
  if (chan == 0xFFFF) {
    return;
  }

  if (spektrum.tx_type == SPEKTRUM_SYS_22_1024_2) {
    // We got a 10bit precision packet
    uint8_t chan_num = (chan & 0xFC00) >> 10;
    sat->values[chan_num] = chan & 0x03FF;
    if (chan_num == RADIO_THROTTLE) {
      // scale full range to pprz_t
      // but since 1024 correspond to 150%, scale with 1024/1.5 ~ 684
      // remove an offset of 2400 = 171 * MAX_PPRZ / 684
      sat->values[chan_num] = ((MAX_PPRZ * sat->values[chan_num]) / 684) - 2400;
    } else {
      sat->values[chan_num] -= (1 << 9); // substract 2^9 to get a value between [-512;512]
      // scale full range to pprz_t
      // but since 512 correspond to 150%, scale with 512/1.5 ~ 342
      sat->values[chan_num] = (MAX_PPRZ * sat->values[chan_num]) / 342;
    }
  }
  else {
    // We got a 11bit precision packet
    uint8_t chan_num = (chan & 0x7800) >> 11;
    sat->values[chan_num] = chan & 0x07FF;
    if (chan_num == RADIO_THROTTLE) {
      // scale full range to pprz_t
      // but since 2048 correspond to 150%, scale with 2048/1.5 ~ 1368
      // remove an offset of 2400 = 234 * MAX_PPRZ / 1368
      sat->values[chan_num] = ((MAX_PPRZ * sat->values[chan_num]) / 1368) - 2400;
    } else {
      sat->values[chan_num] -= (1 << 10); // substract 2^10 to get a value between [-1024;1024]
      // scale full range to pprz_t
      // but since 1024 correspond to 150%, scale with 1024/1.5 ~ 684
      sat->values[chan_num] = (MAX_PPRZ * sat->values[chan_num]) / 684;
    }
  }

  // mark a valid frame
  sat->valid = true;
  spektrum.valid = true;
}

/* Spektrum parser for a satellite */
static inline void spektrum_parser(struct spektrum_sat_t *sat)
{
  // Parse packet
  sat->lost_frame_cnt = sat->buf[0];
  // For now ignore the second byte (which could be the TX type)
  // if frame type is still unknown, try to find it in the 'system' byte
  // only the primary receiver should have a valid type
  if (spektrum.tx_type == 0) {
    uint8_t type = sat->buf[1];
    if (type == SPEKTRUM_SYS_22_1024_2 || type == SPEKTRUM_SYS_11_2048_2 ||
        type == SPEKTRUM_SYS_22_2048_X || type == SPEKTRUM_SYS_11_2048_X) {
      // we have a valid type, we assume it comes from primary receiver
      spektrum.tx_type = type;
    } else {
      // return and drop frame as we don't know what to do with it
      return;
    }
  }
  // parse servo channels
  for (uint8_t i = 2; i < 2*SPEKTRUM_CHANNELS_PER_FRAME+2; i = i+2) {
    uint16_t chan = (((uint16_t)sat->buf[i]) << 8) | ((uint16_t)sat->buf[i+1]);
    spektrum_parse_channel(sat, chan);
  }
}

/** Check bytes on the UART */
static void spektrum_uart_check(struct uart_periph *dev, struct spektrum_sat_t *sat)
{
  // detect sync space based on frame spacing
  uint32_t t = get_sys_time_msec();
  if (t - sat->timer > SPEKTRUM_MIN_FRAME_SPACE) {
    sat->timer = t; // reset counter
    uint16_t bytes_cnt = uart_char_available(dev); // The amount of bytes in the buffer
    // sync space detected but buffer not empty, flush data
    if (bytes_cnt > 0) {
      for (uint8_t i = 0; i < bytes_cnt; i++) {
        uart_getch(dev);
      }
    }
  }
  // check and parse bytes in uart buffer
  while (uart_char_available(dev)) {
    sat->buf[sat->idx++] = uart_getch(dev);
    sat->timer = t; // reset counter
    if (sat->idx == SPEKTRUM_FRAME_LEN) {
      // buffer is full, parse frame
      spektrum_parser(sat);
      sat->idx = 0; // reset index
      break; // stop here to handle RC frame
    }
  }
}

/** Checks if there is one valid satellite and sets the radio_control structure */
void spektrum_event(void)
{
  spektrum_uart_check(&SPEKTRUM_PRIMARY_UART, &spektrum.satellites[0]);

#ifdef SPEKTRUM_SECONDARY_UART
  spektrum_uart_check(&SPEKTRUM_SECONDARY_UART, &spektrum.satellites[1]);
#endif

  // Whenever we received a valid RC packet
  if (spektrum.valid) {
    uint8_t sat_id = SPEKTRUM_SATELLITES_NB;
    spektrum.valid = false;

    // Find the first satellite that has a valid packet
    for (uint8_t i = 0; i < SPEKTRUM_SATELLITES_NB; i++) {
      if (i < sat_id) {
        sat_id = i;
      }
      if (spektrum.satellites[i].valid) {
        spektrum.satellites[i].valid = false;
      }
    }

    // Failsafe case if found satellite is out of bound (Should not happen)
    if (sat_id >= SPEKTRUM_SATELLITES_NB) {
      return;
    }

    // Set the radio control status
    radio_control.frame_cpt++;
    radio_control.time_since_last_frame = 0;
    radio_control.status = RC_OK;

    // Copy the radio control channels
    for (uint8_t i = 0; i < radio_control.nb_channel; i++) {
      radio_control.values[i] = spektrum.satellites[sat_id].values[i] * spektrum.signs[i];
      Bound(radio_control.values[i], -MAX_PPRZ, MAX_PPRZ);
    }

    // We got a valid frame so execute the frame handler
    AbiSendMsgRADIO_CONTROL(RADIO_CONTROL_SPEKTRUM_ID, &radio_control);
  }
}

/* Defines needed for easy access of port information */
#define _UART_RX_PORT(i) i ## _PORT_RX
#define UART_RX_PORT(i) _UART_RX_PORT(i)
#define _UART_RX(i) i ## _RX
#define UART_RX(i) _UART_RX(i)

/**
 * By default, the same pin is used for pulse train and uart rx, but
 * they can be different if needed
 */
#ifndef SPEKTRUM_PRIMARY_BIND_CONF_PORT
#define SPEKTRUM_PRIMARY_BIND_CONF_PORT UART_RX_PORT(SPEKTRUM_PRIMARY_UART_UPPER)
#endif
#ifndef SPEKTRUM_PRIMARY_BIND_CONF_PIN
#define SPEKTRUM_PRIMARY_BIND_CONF_PIN UART_RX(SPEKTRUM_PRIMARY_UART_UPPER)
#endif
#ifndef SPEKTRUM_SECONDARY_BIND_CONF_PORT
#define SPEKTRUM_SECONDARY_BIND_CONF_PORT UART_RX_PORT(SPEKTRUM_SECONDARY_UART_UPPER)
#endif
#ifndef SPEKTRUM_SECONDARY_BIND_CONF_PIN
#define SPEKTRUM_SECONDARY_BIND_CONF_PIN UART_RX(SPEKTRUM_SECONDARY_UART_UPPER)
#endif

/** This function puts the satellite in binding mode.
 * The requirement of this are that this needs to be done while powering up.
 */
static void UNUSED spektrum_bind(void)
{

  /* Master receiver Rx push-pull */
  gpio_setup_output(SPEKTRUM_PRIMARY_BIND_CONF_PORT, SPEKTRUM_PRIMARY_BIND_CONF_PIN);
  /* Master receiver RX line, drive high */
  gpio_set(SPEKTRUM_PRIMARY_BIND_CONF_PORT, SPEKTRUM_PRIMARY_BIND_CONF_PIN);

#ifdef SPEKTRUM_SECONDARY_UART
  /* Slave receiver Rx push-pull */
  gpio_setup_output(SPEKTRUM_SECONDARY_BIND_CONF_PORT, SPEKTRUM_SECONDARY_BIND_CONF_PIN);
  /* Slave receiver RX line, drive high */
  gpio_set(SPEKTRUM_SECONDARY_BIND_CONF_PORT, SPEKTRUM_SECONDARY_BIND_CONF_PIN);
#endif

  /* bind pulses should be issued within 200ms after power up
   * wait a bit to let the receiver start properly,
   * set to 0 to disable */
#if SPEKTRUM_BIND_WAIT
  sys_time_usleep(SPEKTRUM_BIND_WAIT);
#endif

  /* Transmit the bind pulses */
  for (int i = 0; i < 2 * SPEKTRUM_MASTER_RECEIVER_PULSES ; i++) {
    gpio_toggle(SPEKTRUM_PRIMARY_BIND_CONF_PORT, SPEKTRUM_PRIMARY_BIND_CONF_PIN);
    sys_time_usleep(120);
  }
#ifdef SPEKTRUM_SECONDARY_UART
  for (int i = 0; i < 2 * SPEKTRUM_SLAVE_RECEIVER_PULSES; i++) {
    gpio_toggle(SPEKTRUM_SECONDARY_BIND_CONF_PORT, SPEKTRUM_SECONDARY_BIND_CONF_PIN);
    sys_time_usleep(120);
  }
#endif

  /* Set conf pin as input in case it is different from RX pin */
  gpio_setup_input(SPEKTRUM_PRIMARY_BIND_CONF_PORT, SPEKTRUM_PRIMARY_BIND_CONF_PIN);
#ifdef SPEKTRUM_SECONDARY_UART
  gpio_setup_input(SPEKTRUM_SECONDARY_BIND_CONF_PORT, SPEKTRUM_SECONDARY_BIND_CONF_PIN);
#endif
}

