/*
 * Copyright (C) 2015 The Paparazzi Team
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

/** @file subsystems/intermcu/intermcu_fbw.c
 *  @brief Rotorcraft Inter-MCU on FlyByWire
 */

#include "intermcu_fbw.h"
#include "pprzlink/intermcu_msg.h"
#include "subsystems/radio_control.h"
#include "mcu_periph/uart.h"
#include "pprzlink/pprz_transport.h"
#include "modules/spektrum_soft_bind/spektrum_soft_bind_fbw.h"

#ifdef BOARD_PX4IO
#include "libopencm3/cm3/scb.h"
#include "led.h"
#include "mcu_periph/sys_time.h"
static uint8_t px4RebootSequence[] = {0x41, 0xd7, 0x32, 0x0a, 0x46, 0x39};
static uint8_t px4RebootSequenceCount = 0;
static bool_t px4RebootTimeout = FALSE;
uint8_t autopilot_motors_on = FALSE;
tid_t px4bl_tid; ///< id for time out of the px4 bootloader reset
#endif

#if RADIO_CONTROL_NB_CHANNEL > 8
#undef RADIO_CONTROL_NB_CHANNEL
#define RADIO_CONTROL_NB_CHANNEL 8
#warning "INTERMCU UART WILL ONLY SEND 8 RADIO CHANNELS"
#endif

// Used for communication
static struct link_device *intermcu_device = (&((INTERMCU_LINK).device));
static struct pprz_transport intermcu_transport;

struct intermcu_t inter_mcu;
pprz_t intermcu_commands[COMMANDS_NB];
static inline void intermcu_parse_msg(struct transport_rx *trans, void (*commands_frame_handler)(void));
static inline void checkPx4RebootCommand(unsigned char b);

void intermcu_init(void)
{
  pprz_transport_init(&intermcu_transport);
#ifdef BOARD_PX4IO
  px4bl_tid = sys_time_register_timer(20.0, NULL);
#endif

}

void intermcu_periodic(void)
{
  /* Check for interMCU loss */
  if (inter_mcu.time_since_last_frame >= INTERMCU_LOST_CNT) {
    inter_mcu.status = INTERMCU_LOST;
  } else {
    inter_mcu.time_since_last_frame++;
  }
}

void intermcu_on_rc_frame(void)
{
  pprz_msg_send_IMCU_RADIO_COMMANDS(&(intermcu_transport.trans_tx), intermcu_device,
                                    INTERMCU_FBW, 0, RADIO_CONTROL_NB_CHANNEL, radio_control.values); //TODO: Fix status
}

void intermcu_send_status(uint8_t mode)
{
  // Send Status
  (void)mode;
  //FIXME
}

static inline void intermcu_parse_msg(struct transport_rx *trans, void (*commands_frame_handler)(void))
{
  /* Parse the Inter MCU message */
  uint8_t msg_id = trans->payload[1];
  switch (msg_id) {
    case DL_IMCU_COMMANDS: {
      uint8_t i;
      uint8_t size = DL_IMCU_COMMANDS_values_length(trans->payload);
      int16_t *new_commands = DL_IMCU_COMMANDS_values(trans->payload);
      uint8_t status = DL_IMCU_COMMANDS_status(trans->payload);
      autopilot_motors_on = status & 0x1;
      for (i = 0; i < size; i++) {
        intermcu_commands[i] = new_commands[i];
      }

      inter_mcu.status = INTERMCU_OK;
      inter_mcu.time_since_last_frame = 0;
      commands_frame_handler();
      break;
    }

#if defined(SPEKTRUM_HAS_SOFT_BIND_PIN) //TODO: make subscribable module parser
    case DL_IMCU_SPEKTRUM_SOFT_BIND:
      received_spektrum_soft_bind();
      break;
#endif
    default:
      break;
  }

  // Set to receive another message
  trans->msg_received = FALSE;
}

void InterMcuEvent(void (*frame_handler)(void))
{

  /* Parse incoming bytes */
  if (intermcu_device->char_available(intermcu_device->periph)) {
    while (intermcu_device->char_available(intermcu_device->periph) && !intermcu_transport.trans_rx.msg_received) {
      unsigned char b = intermcu_device->get_byte(intermcu_device->periph);
#ifdef BOARD_PX4IO
      checkPx4RebootCommand(b);
#endif
      parse_pprz(&intermcu_transport, b);
    }

    if (intermcu_transport.trans_rx.msg_received) {
      intermcu_parse_msg(&(intermcu_transport.trans_rx), frame_handler);
    }
  }
}
#ifdef BOARD_PX4IO
static inline void checkPx4RebootCommand(unsigned char b)
{
  if (!px4RebootTimeout) {


    if (sys_time_check_and_ack_timer(px4bl_tid)) {
      //time out the possibility to reboot to the px4 bootloader, to prevent unwanted restarts during flight
      px4RebootTimeout = true;
      sys_time_cancel_timer(px4bl_tid);
      return;
    }

    LED_ON(SYS_TIME_LED);

    if (b == px4RebootSequence[px4RebootSequenceCount]) {
      px4RebootSequenceCount++;
    } else {
      px4RebootSequenceCount = 0;
    }

    if (px4RebootSequenceCount >= 6) { // 6 = length of rebootSequence + 1
      px4RebootSequenceCount = 0; // should not be necessary...

      //send some magic back
      //this is the same as the Pixhawk IO code would send
      intermcu_device->put_byte(intermcu_device->periph, 0x00);
      intermcu_device->put_byte(intermcu_device->periph, 0xe5);
      intermcu_device->put_byte(intermcu_device->periph, 0x32);
      intermcu_device->put_byte(intermcu_device->periph, 0x0a);
      intermcu_device->put_byte(intermcu_device->periph,
                                0x66); // dummy byte, seems to be necessary otherwise one byte is missing at the fmu side...

      while (((struct uart_periph *)(intermcu_device->periph))->tx_running) {LED_TOGGLE(SYS_TIME_LED);}  // tx_running is volatile now, so LED_TOGGLE not necessary anymore

      LED_OFF(SYS_TIME_LED);
      scb_reset_system();
    }
  }
}
#endif
