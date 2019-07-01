/*
 * Copyright (C) 2017  Michal Podhradsky
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

/** \file modules/fsae_electric/vms_ecu_demo.c
 *
 * Viking Motorsports Engine Control Unit demo module
 * see https://wiki.paparazziuav.org/wiki/VMS_ECU
 * for more details
 */
#include "modules/fsae_electric/vms_ecu_demo.h"

// Messages
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#include "mcu_periph/gpio.h"
#include "ch.h" // for DAC
#include "hal.h" // for DAC

#include "generated/airframe.h"
#include BOARD_CONFIG

/*
 * ADC
 */
// define bit resolution
#ifndef ADC_BIT_RES
#define ADC_BIT_RES 12
#endif

// default reference (3.3 Volts)
#define ADC_VREF 3300.0 //mV

// reading multiplier
#define ADC_VREF_MULT  ADC_VREF/(1<<ADC_BIT_RES)

// voltage divider value
#define ADC_VGAIN 1.5 // 1k1 and 2k2 (5V->3.3V)

#define DAC_BUFFER_SIZE 1//360

float ain_1;
float ain_2;
float ain_3;
float ain_4;

struct adc_buf adc_buf_1;
struct adc_buf adc_buf_2;
struct adc_buf adc_buf_3;
struct adc_buf adc_buf_4;



/*
 * DAC
 */
uint16_t dac_1;
uint16_t dac_2;

static const DACConfig dac1cfg1 = {
    .init         = 2047U,
    .datamode     = DAC_DHRM_12BIT_RIGHT
};

static const DACConfig dac1cfg2 = {
    .init         = 0U,
    .datamode     = DAC_DHRM_12BIT_RIGHT
};


dacsample_t dac_ref1;
dacsample_t dac_ref2;


/*
 * DIGITAL IO
 */
bool ams_status;
bool pwr_ready;
bool pwr_stdby;
bool rtds;

uint8_t stg_in;
uint8_t stb_in;



/*
 * CAN
 */
struct can_instance {
  CANDriver     *canp;
  uint32_t      led;
};

static const struct can_instance can1 = {&CAND1, 11};
static const struct can_instance can2 = {&CAND2, 12};


/*
 * Internal loopback mode, 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */
// static const CANConfig cancfg_lb = {
//     CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
//     CAN_BTR_LBKM | CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
//     CAN_BTR_TS1(8) | CAN_BTR_BRP(6)
// };

/*
 * Normal mode, see if we can ping each other
 */
static const CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
    CAN_BTR_TS1(8) | CAN_BTR_BRP(6)
};

/*
 * Receiver thread.
 */
static THD_WORKING_AREA(can_rx1_wa, 256);
static THD_WORKING_AREA(can_rx2_wa, 256);
static THD_FUNCTION(can_rx, p) {
  struct can_instance *cip = p;
  event_listener_t el;
  CANRxFrame rxmsg;

  (void)p;
  chRegSetThreadName("receiver");
  chEvtRegister(&cip->canp->rxfull_event, &el, 0);
  while(!chThdShouldTerminateX()) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0)
      continue;
    while (canReceive(cip->canp, CAN_ANY_MAILBOX,
        &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
      // Process message.
      palTogglePad(GPIOD, cip->led);
    }
  }
  //chEvtUnregister(&CAND1.rxfull_event, &el);
  chEvtUnregister(&cip->canp->rxfull_event, &el);
}


/*
 * Transmitter thread.
 */
static THD_WORKING_AREA(can_tx_wa, 256);
static THD_FUNCTION(can_tx, p) {
  CANTxFrame txmsg;

  (void)p;
  chRegSetThreadName("transmitter");
  txmsg.IDE = CAN_IDE_EXT;
  txmsg.EID = 0x01234567;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 8;
  txmsg.data32[0] = 0x55AA55AA;
  txmsg.data32[1] = 0x00FF00FF;

  while (!chThdShouldTerminateX()) {
    //canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(100));
    canTransmit(&CAND2, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(100));
    chThdSleepMilliseconds(500);
  }
}


void vms_ecu_demo_init(void)
{
  // Digital
  ams_status = false;
  pwr_ready = false;
  pwr_stdby = false;
  rtds = false;
  stg_in = 0;
  stb_in = 0;

  // Analog
  ain_1 = 0.0;
  ain_2 = 0.0;
  ain_3 = 0.0;
  ain_4 = 0.0;

  adc_buf_channel(ADC_1, &adc_buf_1, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_2, &adc_buf_2, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_3, &adc_buf_3, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_4, &adc_buf_4, DEFAULT_AV_NB_SAMPLE);

  dac_1 = 0;
  dac_2 = 0;

  /*
   * Activates the CAN drivers 1 and 2.
   */
  canStart(&CAND1, &cancfg);
  canStart(&CAND2, &cancfg);



  /*
   * Starting the transmitter and receiver threads.
   */
  chThdCreateStatic(can_rx1_wa, sizeof(can_rx1_wa), NORMALPRIO + 7,
      can_rx, (void *)&can1);
  chThdCreateStatic(can_rx2_wa, sizeof(can_rx2_wa), NORMALPRIO + 7,
      can_rx, (void *)&can2);
  chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 7,
      can_tx, NULL);

  //DAC
  /*
   * Starting DAC1 driver, setting up the output pin as analog as suggested
   * by the Reference Manual.
   */
  palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
  dacStart(&DACD1, &dac1cfg1);
  dacStart(&DACD2, &dac1cfg2);
  dac_ref1 = 0;
  dac_ref2 = 0;

  dacPutChannelX(&DACD1,0,dac_ref1);
  dacPutChannelX(&DACD2,1,dac_ref2);
}

void vms_ecu_demo_periodic(void)
{
  //  * PD10 - Digital output. (DOUT_LS1)
  if (ams_status) {
    gpio_set(GPIOD, 10);
  }
  else {
    gpio_clear(GPIOD, 10);
  }

  //  *PD11 - Digital output. (DOUT_LS2).
  if (pwr_ready) {
    gpio_set(GPIOD, 11);
  }
  else {
    gpio_clear(GPIOD, 11);
  }

  //  *PD12 - Digital output. (DOUT_LS3).
  if (pwr_stdby) {
    gpio_set(GPIOD, 12);
  }
  else {
    gpio_clear(GPIOD, 12);
  }

  //  *PD13 - Digital output. (DOUT_LS4).
  static uint8_t cnt = 0;
  if (rtds) {
    // 120Hz, 0-5 Ain
    // 5V -> fast flash
    // 0V -> slow flash
    if (cnt > (10-(uint8_t)(ain_1 + ain_2))) {

      gpio_set(GPIOD, 13);
      cnt = 0;
    }
    else {
      gpio_clear(GPIOD, 13);
      cnt++;
    }


  }
  else {
    gpio_clear(GPIOD, 13);
  }

  static bool flag = false;
  // read inputs
  if (flag) {
    stg_in = palReadPad(GPIOE, 7); //  PE7  - Digital input. (DIN1:DIN_STG1).
    flag = false;
  }
  else {
    stb_in = palReadPad(GPIOE, 9); // PE9 - Digital input. (DIN3:DIN_STB1).
    flag = true;
  }

  // Analog
  ain_1 = ((float)(adc_buf_1.sum/adc_buf_1.av_nb_sample))*ADC_VREF_MULT*ADC_VGAIN/1000.0;
  ain_2 = ((float)(adc_buf_2.sum/adc_buf_2.av_nb_sample))*ADC_VREF_MULT*ADC_VGAIN/1000.0;
  ain_3 = ((float)(adc_buf_3.sum/adc_buf_3.av_nb_sample))*ADC_VREF_MULT*ADC_VGAIN/1000.0;
  ain_4 = ((float)(adc_buf_4.sum/adc_buf_4.av_nb_sample))*ADC_VREF_MULT*ADC_VGAIN/1000.0;
}


void vms_ecu_demo_downlink(void) {
  DOWNLINK_SEND_ECU(DefaultChannel, DefaultDevice,
      &stg_in,
      &stb_in,
      &ain_1,
      &ain_2,
      &ain_3,
      &ain_4);
}


void vms_ecu_demo_UpdateDac1(uint16_t val) {
  dac_1 = val;
  dac_ref1 = dac_1;
  dacPutChannelX(&DACD1,0,dac_ref1);
}

void vms_ecu_demo_UpdateDac2(uint16_t val) {
  dac_2 = val;
  dac_ref2 = dac_2;
  dacPutChannelX(&DACD2,1,dac_ref2);
}
