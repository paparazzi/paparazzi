/*
 * Copyright (C) 2025 The Paparazzi Team
 * 
 * This file is part of paparazzi.
 *
 * Paparazzi is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * See LICENSE file for the full license version, or see http://www.gnu.org/licenses/
 */

#include "mcu_periph/can_arch.h"
#include "mcu_periph/can.h"
#include "mcu_periph/sys_time_arch.h"
#include "stdio.h"
#include "string.h"

#include <ch.h>
#include <hal.h>


struct can_arch_periph {
  int if_index;
  CANDriver* cand;
  CANConfig cfg;
  uint32_t can_baudrate;

  void *thread_rx_wa;
  size_t thread_rx_wa_size;
};

static void can_thd_rx(void* arg);
static void can_start(struct can_periph* canp);
static bool canConfigureIface(struct can_arch_periph* cas);

#if USE_CAN1

static THD_WORKING_AREA(can1_rx_wa, 1024 * 2);

struct can_arch_periph can1_arch_s = {
  .if_index = 1,
  .cand = &CAND1,
  .cfg = {0},
  .can_baudrate = 1000000U,
  .thread_rx_wa = can1_rx_wa,
  .thread_rx_wa_size = sizeof(can1_rx_wa),
};

#endif

#if USE_CAN2

static THD_WORKING_AREA(can2_rx_wa, 1024 * 2);

struct can_arch_periph can2_arch_s = {
  .if_index = 2,
  .cand = &CAND2,
  .cfg = {0},
  .can_baudrate = 1000000U,
  .thread_rx_wa = can2_rx_wa,
  .thread_rx_wa_size = sizeof(can2_rx_wa),
};

#endif

void can_hw_init() {
  #if USE_CAN1
  can1.arch_struct = &can1_arch_s;
  can_start(&can1);
  #endif
  #if USE_CAN2
  can2.arch_struct = &can2_arch_s;
  can_start(&can2);
  #endif
}



static void can_thd_rx(void* arg) {
  struct can_periph* canp = (struct can_periph*)arg;
  struct can_arch_periph* cas = (struct can_arch_periph*)canp->arch_struct;
  char thd_name[10];
  snprintf(thd_name, 10, "can%d_rx", cas->if_index);
  chRegSetThreadName(thd_name);

  struct pprzaddr_can addr = {
    .can_ifindex = cas->if_index
  };

  while(!chThdShouldTerminateX()) {
    CANRxFrame rx_frame;
    msg_t status = canReceiveTimeout(cas->cand, CAN_ANY_MAILBOX, &rx_frame, chTimeMS2I(50));
    if(status == MSG_OK) { 
      uint32_t id = 0;
      if(rx_frame.common.XTD) {
        id = rx_frame.ext.EID | CAN_FRAME_EFF;
      } else {
        id = rx_frame.std.SID;
      }
      if(rx_frame.common.RTR) {
        id |= CAN_FRAME_RTR;
      }
      if(rx_frame.common.ESI) {
        id |= CAN_FRAME_ERR;
      }

      struct pprzcan_frame pprz_frame = {
        .can_id = id,
        .len = can_dlc_to_len(rx_frame.DLC),
        .flags = 0,
        .timestamp = get_sys_time_msec(),
      };
      
      if(rx_frame.FDF) {
        pprz_frame.flags |= CANFD_FDF;
      }
      if(rx_frame.common.ESI) {
        pprz_frame.flags |= CANFD_ESI;
      }



      memcpy(pprz_frame.data, rx_frame.data8, pprz_frame.len);

      for(int i=0; i<CAN_NB_CALLBACKS_MAX; i++) {
        if(canp->callbacks[i] != NULL) {
          canp->callbacks[i](&pprz_frame, &addr, canp->callback_user_data[i]);
        }
      }
    }
  }

}

int can_transmit_frame(struct pprzcan_frame* txframe, struct pprzaddr_can* addr) {
  CANTxFrame frame;
  frame.DLC = can_len_to_dlc(txframe->len);
  if(txframe->can_id & CAN_FRAME_RTR) {
    frame.common.RTR = 1;
  }
  if(txframe->can_id & CAN_FRAME_EFF) {
    frame.common.XTD = 1;
    frame.ext.EID = txframe->can_id & CAN_EID_MASK
  } else {
    frame.std.SID = txframe->can_id & CAN_SID_MASK
  }
  memcpy(frame.data8, txframe->data, txframe->len);

  #if USE_CAN1
  if(addr->can_ifindex == 1 || addr->can_ifindex == 0) {
    msg_t ret = canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &frame, TIME_IMMEDIATE);
    if(ret != MSG_OK) {
      return ret;
    }
  }
  #endif

  #if USE_CAN2
  if(addr->can_ifindex == 2 || addr->can_ifindex == 0) {
    msg_t ret = canTransmitTimeout(&CAND2, CAN_ANY_MAILBOX, &frame, TIME_IMMEDIATE);
    if(ret != MSG_OK) {
      return ret;
    }
  }
  #endif
  
  return 0;
}

static void can_start(struct can_periph* canp) {
  struct can_arch_periph* cas = (struct can_arch_periph*)canp->arch_struct;

  #if defined(STM32_CAN_USE_FDCAN1) || defined(STM32_CAN_USE_FDCAN2)
  // Configure the RAM
  can1_arch_s.cfg.RXF0C = (32 << FDCAN_RXF0C_F0S_Pos) | (0 << FDCAN_RXF0C_F0SA_Pos);
  can1_arch_s.cfg.RXF1C = (32 << FDCAN_RXF1C_F1S_Pos) | (128 << FDCAN_RXF1C_F1SA_Pos);
  can1_arch_s.cfg.TXBC  = (32 << FDCAN_TXBC_TFQS_Pos) | (256 << FDCAN_TXBC_TBSA_Pos);
  can1_arch_s.cfg.TXESC = 0x000; // 8 Byte mode only (4 words per message)
  can1_arch_s.cfg.RXESC = 0x000; // 8 Byte mode only (4 words per message)
  #endif
  if (!canConfigureIface(cas)) {
    return;
  }


  canStart(cas->cand, &can1_arch_s.cfg);
  chThdCreateStatic(cas->thread_rx_wa, cas->thread_rx_wa_size,
                    NORMALPRIO + 8, can_thd_rx, canp);
}


/**
 * Try to compute the timing registers for the can interface and set the configuration
 */
static bool canConfigureIface(struct can_arch_periph* cas)
{
  if (cas->can_baudrate < 1) {
    return false;
  }

  // Hardware configurationn
#if defined(STM32_CAN_USE_FDCAN1) || defined(STM32_CAN_USE_FDCAN2)
  const uint32_t pclk = STM32_FDCANCLK;
#else
  const uint32_t pclk = STM32_PCLK1;
#endif
  static const int MaxBS1 = 16;
  static const int MaxBS2 = 8;

  /*
    * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
    *      CAN in Automation, 2003
    *
    * According to the source, optimal quanta per bit are:
    *   Bitrate        Optimal Maximum
    *   1000 kbps      8       10
    *   500  kbps      16      17
    *   250  kbps      16      17
    *   125  kbps      16      17
    */
  const int max_quanta_per_bit = (cas->can_baudrate >= 1000000) ? 10 : 17;
  static const int MaxSamplePointLocation = 900;

  /*
    * Computing (prescaler * BS):
    *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
    *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
    * let:
    *   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
    *   PRESCALER_BS = PRESCALER * BS
    * ==>
    *   PRESCALER_BS = PCLK / BITRATE
    */
  const uint32_t prescaler_bs = pclk / cas->can_baudrate;

// Searching for such prescaler value so that the number of quanta per bit is highest.
  uint8_t bs1_bs2_sum = max_quanta_per_bit - 1;
  while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
    if (bs1_bs2_sum <= 2) {
      return false;          // No solution
    }
    bs1_bs2_sum--;
  }

  const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
  if ((prescaler < 1U) || (prescaler > 1024U)) {
    return false;              // No solution
  }

  /*
    * Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
    * We need to find the values so that the sample point is as close as possible to the optimal value.
    *
    *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
    *   {{bs2 -> (1 + bs1)/7}}
    *
    * Hence:
    *   bs2 = (1 + bs1) / 7
    *   bs1 = (7 * bs1_bs2_sum - 1) / 8
    *
    * Sample point location can be computed as follows:
    *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
    *
    * Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
    *   - With rounding to nearest
    *   - With rounding to zero
    */
// First attempt with rounding to nearest
  uint8_t bs1 = ((7 * bs1_bs2_sum - 1) + 4) / 8;
  uint8_t bs2 = bs1_bs2_sum - bs1;
  uint16_t sample_point_permill = 1000 * (1 + bs1) / (1 + bs1 + bs2);

// Second attempt with rounding to zero
  if (sample_point_permill > MaxSamplePointLocation) {
    bs1 = (7 * bs1_bs2_sum - 1) / 8;
    bs2 = bs1_bs2_sum - bs1;
    sample_point_permill = 1000 * (1 + bs1) / (1 + bs1 + bs2);
  }

  /*
    * Final validation
    * Helpful Python:
    * def sample_point_from_btr(x):
    *     assert 0b0011110010000000111111000000000 & x == 0
    *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
    *     return (1+ts1+1)/(1+ts1+1+ts2+1)
    *
    */
  if ((cas->can_baudrate != (pclk / (prescaler * (1 + bs1 + bs2)))) || (bs1 < 1) || (bs1 > MaxBS1) || (bs2 < 1)
      || (bs2 > MaxBS2)) {
    return false;
  }

  // Configure the interface
#if defined(STM32_CAN_USE_FDCAN1) || defined(STM32_CAN_USE_FDCAN2)
  cas->cfg.NBTP = (0 << FDCAN_NBTP_NSJW_Pos) | ((bs1 - 1) << FDCAN_NBTP_NTSEG1_Pos) | ((
                          bs2 - 1) << FDCAN_NBTP_NTSEG2_Pos) | ((prescaler - 1) << FDCAN_NBTP_NBRP_Pos);
  #if USE_CANFD
    cas->cfg.CCCR = FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE;
  #else
    cas->cfg.CCCR = 0;
  #endif

#else
  cas->cfg.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
  cas->cfg.btr = CAN_BTR_SJW(0) | CAN_BTR_TS1(bs1 - 1) | CAN_BTR_TS2(bs2 - 1) | CAN_BTR_BRP(prescaler - 1);
#endif
  return true;
}