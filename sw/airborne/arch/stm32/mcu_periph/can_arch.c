/*
 * Copyright (C) 2012 Piotr Esden-Tempski <piotr@esden.net>
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
 * @file arch/stm32/mcu_periph/can_arch.c
 * @ingroup stm32_arch
 *
 * Handling of CAN hardware for STM32.
 */

#include <stdint.h>
#include <string.h>

#include "mcu_periph/can_arch.h"
#include "mcu_periph/can.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/cm3/nvic.h>

#include "led.h"


#ifdef RTOS_PRIO
#define NVIC_USB_LP_CAN_RX0_IRQ_PRIO RTOS_PRIO+1
#else
#define NVIC_USB_LP_CAN_RX0_IRQ_PRIO 1
#define NVIC_CAN1_RX_IRQ_PRIO 1
#endif

void _can_run_rx_callback(uint32_t id, uint8_t *buf, uint8_t len);

bool can_initialized = false;

void can_hw_init(void)
{


#ifdef STM32F1
  /* Enable peripheral clocks. */
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_CAN1);

  /* Remap the gpio pin if necessary. */
  AFIO_MAPR |= AFIO_MAPR_CAN1_REMAP_PORTB;

  /* Configure CAN pin: RX (input pull-up). */
  gpio_set_mode(GPIO_BANK_CAN1_PB_RX, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN1_PB_RX);
  gpio_set(GPIO_BANK_CAN1_PB_RX, GPIO_CAN1_PB_RX);

  /* Configure CAN pin: TX (output push-pull). */
  gpio_set_mode(GPIO_BANK_CAN1_PB_TX, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN1_PB_TX);

  /* NVIC setup. */
  nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
  nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, NVIC_USB_LP_CAN_RX0_IRQ_PRIO);

#elif STM32F4

  /* Enable peripheral clocks. */
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_CAN1);

  /* set up pins for CAN1TX & CAN1RX alternate function */
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
  gpio_set_af(GPIOB, GPIO_AF9, GPIO8 | GPIO9);

  /* enable interrupts on RX0 FIFO */
  nvic_enable_irq(NVIC_CAN1_RX0_IRQ);
  nvic_set_priority(NVIC_CAN1_RX0_IRQ, NVIC_CAN1_RX_IRQ_PRIO);

#endif
  /* Reset CAN. */
  can_reset(CAN1);

  /* CAN cell init.
   * For time quanta calculation see STM32 reference manual
   * section 24.7.7 "Bit timing" page 645
   *
   * To talk to CSC using LPC mcu we need a baud rate of 375kHz
   * The APB1 runs at 36MHz therefor we select a prescaler of 12
   * resulting in time quanta frequency of 36MHz / 12 = 3MHz
   *
   * As the Bit time is combined of 1tq for SYNC_SEG, TS1tq for bit
   * segment 1 and TS2tq for bit segment 2:
   * BITtq = 1tq + TS1tq + TS2tq
   *
   * We can choose to use TS1 = 3 and TS2 = 4 getting
   * 1tq + 3tq + 4tq = 8tq per bit therefor a bit frequency is
   * 3MHZ / 8 = 375kHz
   *
   * Maximum baud rate of CAN is 1MHz so we can choose to use
   * prescaler of 2 resulting in a quanta frequency of 36MHz / 2 = 18Mhz
   *
   * So we need to devide the frequency by 18. This can be accomplished
   * using TS1 = 10 and TS2 = 7 resulting in:
   * 1tq + 10tq + 7tq = 18tq
   *
   * NOTE: Although it is out of spec I managed to have CAN run at 2MBit
   * Just decrease the prescaler to 1. It worked for me(tm) (esden)
   */
  if (can_init(CAN1,
               false,           /* TTCM: Time triggered comm mode? */
               true,            /* ABOM: Automatic bus-off management? */
               false,           /* AWUM: Automatic wakeup mode? */
               false,           /* NART: No automatic retransmission? */
               false,           /* RFLM: Receive FIFO locked mode? */
               false,           /* TXFP: Transmit FIFO priority? */
#ifdef STM32F1
               CAN_BTR_SJW_1TQ,
               CAN_BTR_TS1_10TQ,
               CAN_BTR_TS2_7TQ,
#elif STM32F4
               CAN_BTR_SJW_1TQ,
               CAN_BTR_TS1_14TQ,
               CAN_BTR_TS2_6TQ,
#endif
               2,               /* BRP+1: Baud rate prescaler */
               false,           /* loopback mode */
               false)) {        /* silent mode */
    /* TODO we need something somewhere where we can leave a note
     * that CAN was unable to initialize. Just like any other
     * driver should...
     */

    can_reset(CAN1);

    return;
  }

  /* CAN filter 0 init. */
  can_filter_id_mask_32bit_init(0,     /* Filter ID */
                                0,     /* CAN ID */
                                0,     /* CAN ID mask */
                                0,     /* FIFO assignment (here: FIFO0) */
                                true); /* Enable the filter. */

  /* Enable CAN RX interrupt. */
  can_enable_irq(CAN1, CAN_IER_FMPIE0);

  /* Remember that we succeeded to initialize. */
  can_initialized = true;
}

int can_hw_transmit(uint32_t id, const uint8_t *buf, uint8_t len)
{

  if (!can_initialized) {
    return -2;
  }

  if (len > 8) {
    return -1;
  }


  /* FIXME: we are discarding the const qualifier for buf here.
   * We should probably fix libopencm3 to actually have the
   * const qualifier too...
   */
  return can_transmit(CAN1,
                      id,     /* (EX/ST)ID: CAN ID */
#ifdef USE_CAN_EXT_ID
                      true,  /* IDE: CAN ID extended */
#else
                      false, /* IDE: CAN ID not extended */
#endif
                      false, /* RTR: Request transmit? */
                      len,   /* DLC: Data length */
                      (uint8_t *)buf);
}
#ifdef STM32F1
void usb_lp_can_rx0_isr(void)
{
  uint32_t id;
  uint8_t fmi;
  bool ext, rtr;
  uint8_t length, data[8];
  uint16_t timestamp;

  can_receive(CAN1,
              0,     /* FIFO: 0 */
              false, /* Release */
              &id,
              &ext,
              &rtr,
              &fmi,
              &length,
              data,
              &timestamp);

  _can_run_rx_callback(id, data, length);

  can_fifo_release(CAN1, 0);
}
#elif STM32F4
void can1_rx0_isr(void){
  uint32_t id;
  uint8_t fmi;
  bool ext, rtr;
  uint8_t length, data[8];
  uint16_t timestamp;

  can_receive(CAN1,
              0,     /* FIFO: 0 */
              false, /* Release */
              &id,
              &ext,
              &rtr,
              &fmi,
              &length,
              data,
              &timestamp);

  _can_run_rx_callback(id, data, length);

  can_fifo_release(CAN1, 0);
}
#endif
