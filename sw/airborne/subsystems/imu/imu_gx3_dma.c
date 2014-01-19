/*
 * Copyright (C) 2013 Michal Podhradsky
 * Utah State University, http://aggieair.usu.edu/
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
 * @file imu_gx3.c
 *
 * DMA Driver for Microstrain GX3 IMU/AHRS subsystem
 *
 * Takes care of configuration of the IMU, communication and parsing
 * the received packets. See GX3 datasheet for configuration options.
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#include "subsystems/imu/imu_gx3.h"

#define GX3_CHKSM(_ubx_payload) (uint16_t)((uint16_t)(*((uint8_t*)_ubx_payload+66))|(uint16_t)(*((uint8_t*)_ubx_payload+65))<<8)

#if !USE_CHIBIOS_RTOS
#error "GX3_DMA subsystem is for RT Paparazzi only. Please use non-dma gx3 subsystem instead."
#endif

/*
 * Telemetry defines
 */
#if DOWNLINK
#include "subsystems/datalink/telemetry.h"
static void send_gx3_info(void) {
DOWNLINK_SEND_GX3_INFO(DefaultChannel, DefaultDevice,
  &imu_gx3.gx3_freq,
  &imu_gx3.ch_freq,
  &imu_gx3.gx3_packet.chksm_error,
  &imu_gx3.gx3_packet.hdr_error,
  &imu_gx3.queue.length,
  &imu_gx3.freq_err,
  &imu_gx3.gx3_chksm);
}
#endif /* DOWNLINK */

struct ImuGX3 imu_gx3;

static inline bool_t gx3_verify_chk(volatile uint8_t *buff_add);
static inline float bef(volatile uint8_t *c);

/* Big Endian to Float */
static inline float bef(volatile uint8_t *c) {
  float f;
  int8_t * p;
  p = ((int8_t *)&f)+3;
  *p-- = *c++;
  *p-- = *c++;
  *p-- = *c++;
  *p = *c;
  return f;
}

static inline bool_t gx3_verify_chk(volatile uint8_t *buff_add) {
  uint16_t i,chk_calc;
  chk_calc = 0;
  for (i=0;i<GX3_MSG_LEN-2;i++) {
    chk_calc += (uint8_t)*buff_add++;
  }
  return (chk_calc == ( (((uint16_t)*buff_add)<<8) + (uint8_t)*(buff_add+1) ));
}


void imu_align(void) {
  imu_gx3.gx3_status = GX3Uninit;
  // make the gyros zero, takes 10s (specified in Byte 4 and 5)
  uint8_t imu_buf[5] = {0};
  imu_buf[0] = 0xcd;
  imu_buf[1] = 0xc1;
  imu_buf[2] = 0x29;
  imu_buf[3] = 0x27;
  imu_buf[4] = 0x10;
  uartStartSend(&GX3_DMA_PORT, (size_t) 5, imu_buf);
  imu_gx3.gx3_status = GX3Running;
}

/*
 * GX3 can be set up during the startup, or it can be configured to
 * start sending data automatically after power up.
 */
void imu_impl_init(void) {
  // Initialize variables
  imu_gx3.gx3_status = GX3Uninit;

  // Initialize packet
  imu_gx3.gx3_packet.status = GX3PacketWaiting;
  imu_gx3.gx3_packet.msg_idx = 0;
  imu_gx3.gx3_packet.msg_available = FALSE;
  imu_gx3.gx3_packet.chksm_error = 0;
  imu_gx3.gx3_packet.hdr_error = 0;

  // It is necessary to wait for GX3 to power up for proper initialization
  sys_time_usleep(500000);

  memset(imu_gx3.gx3_data_buffer, 0, GX3_MSG_LEN);
  imu_gx3.queue.rear = -1;
  imu_gx3.queue.front = 0;
  imu_gx3.queue.length = 0;
  imu_gx3.freq_err = 0;
  imu_gx3.data_valid = FALSE;

#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, "GX3_INFO", send_gx3_info);
#endif
}

/*
 * IMU periodic function
 */
void imu_periodic(void) {
  static uint8_t imu_buf[20] = {0};
  if (imu_gx3.gx3_status == GX3Running) {
    // We are initialized already
    // Get oldest packet from the queue
    chMtxLock(&imu_get_data_flag);
    if (imu_gx3.queue.length > 2) {
      memcpy(imu_gx3.gx3_data_buffer, &imu_gx3.queue.queue_buf[imu_gx3.queue.front], GX3_MSG_LEN);
      imu_gx3.queue.length--;
      imu_gx3.queue.queue_buf[imu_gx3.queue.front][0] = 0;
      if (imu_gx3.queue.front == GX3_QUEUE_SIZE-1) {
        imu_gx3.queue.front = 0;
      }
      else {
        imu_gx3.queue.front++;
      }
      chMtxUnlock();
      gx3_packet_read_message();
      imu_gx3.data_valid = TRUE;
    }
    else {
      chMtxUnlock();
    }
  }
  else {
    // Send data according to the state machine
    switch(imu_gx3.gx3_status) {
    case GX3Uninit:
      imu_buf[0] = 0xfa;
      imu_buf[1] = 0x75;
      imu_buf[2] = 0xb4;
      uartStartSend(&GX3_DMA_PORT, (size_t) 3, imu_buf);
      imu_gx3.gx3_status = GX3StopContinuousMode;
      break;
    case GX3StopContinuousMode_OK:
      imu_buf[0] = 0xdb;
      imu_buf[1] = 0xa8;
      imu_buf[2] = 0xb9;
      imu_buf[3] = 0x01;//set and dont save
      imu_buf[4] = 0x00;//IMU_DIV1
      imu_buf[5] = 0x02;//IMU_DIV2
      imu_buf[6] = 0b00000000;//settings
      imu_buf[7] = 0b00000011;//settings
      imu_buf[8] = 2;//ACC_FILT_DIV
      imu_buf[9] = 30;//MAG_FILT_DIV
      imu_buf[10] = 0x00;//Up compensation in secs
      imu_buf[11] = 10;//Up compensation in secs
      imu_buf[12] = 0x00;//North compensation in secs
      imu_buf[13] = 10;//North compensation in secs
      imu_buf[14] = 0x00;//power settings
      // the rest is zeros
      uartStartSend(&GX3_DMA_PORT, (size_t) 20, imu_buf);
      imu_gx3.gx3_status = GX3SamplingSettings;
      break;
    case GX3SamplingSettings_OK:
      imu_buf[0] = 0xc4;
      imu_buf[1] = 0xc1;
      imu_buf[2] = 0x29;
      imu_buf[3] = 0xc8;
      uartStartSend(&GX3_DMA_PORT, (size_t) 4, imu_buf);
      imu_gx3.gx3_status = GX3ContinuousMode;
      break;
    default:
      break;
    }
  }
}


/*
 * Read received packet
 */
void gx3_packet_read_message(void) {
  // Read message from the queue buffer
  imu_gx3.ch_time = chTimeNow();
  imuf.accel.x     = bef(&imu_gx3.gx3_data_buffer[1]);
  imuf.accel.y     = bef(&imu_gx3.gx3_data_buffer[5]);
  imuf.accel.z     = bef(&imu_gx3.gx3_data_buffer[9]);
  VECT3_SMUL(imuf.accel, imuf.accel, 9.80665); // Convert g into m/s2
  ACCELS_BFP_OF_REAL(imu.accel, imuf.accel); // for backwards compatibility with fixed point interface

  imuf.gyro.p      = bef(&imu_gx3.gx3_data_buffer[13]);
  imuf.gyro.q      = bef(&imu_gx3.gx3_data_buffer[17]);
  imuf.gyro.r      = bef(&imu_gx3.gx3_data_buffer[21]);

  imu_gx3.rmat.m[0]   = bef(&imu_gx3.gx3_data_buffer[25]);
  imu_gx3.rmat.m[1]   = bef(&imu_gx3.gx3_data_buffer[29]);
  imu_gx3.rmat.m[2]   = bef(&imu_gx3.gx3_data_buffer[33]);
  imu_gx3.rmat.m[3]   = bef(&imu_gx3.gx3_data_buffer[37]);
  imu_gx3.rmat.m[4]   = bef(&imu_gx3.gx3_data_buffer[41]);
  imu_gx3.rmat.m[5]   = bef(&imu_gx3.gx3_data_buffer[45]);
  imu_gx3.rmat.m[6]   = bef(&imu_gx3.gx3_data_buffer[49]);
  imu_gx3.rmat.m[7]   = bef(&imu_gx3.gx3_data_buffer[53]);
  imu_gx3.rmat.m[8]   = bef(&imu_gx3.gx3_data_buffer[57]);

  imu_gx3.gx3_time    = (uint32_t)(imu_gx3.gx3_data_buffer[61] << 24 |
                                   imu_gx3.gx3_data_buffer[62] << 16 |
                                   imu_gx3.gx3_data_buffer[63] << 8  |
                                   imu_gx3.gx3_data_buffer[64]);
  imu_gx3.gx3_chksm   = GX3_CHKSM(imu_gx3.gx3_data_buffer);

  imu_gx3.gx3_freq = 62500.0 / (float)(imu_gx3.gx3_time - imu_gx3.gx3_ltime);
  imu_gx3.ch_freq = CH_FREQUENCY/(imu_gx3.ch_time - imu_gx3.ch_ltime);
  imu_gx3.gx3_ltime = imu_gx3.gx3_time;
  imu_gx3.ch_ltime = imu_gx3.ch_time;

  //TODO: Error checking - just a rough idea for now
  if ((imu_gx3.gx3_freq >= 510) || (imu_gx3.gx3_freq <= 490)) {
    imu_gx3.freq_err++;
  }
}

/*
 * GX3 Packet Collection & state machine
 */
void gx3_packet_parse( uint8_t c) {
  switch (imu_gx3.gx3_packet.status) {
    case GX3PacketWaiting:
      imu_gx3.gx3_packet.msg_idx = 0;
      if (c == GX3_HEADER) {
        imu_gx3.gx3_packet.status = GX3PacketReading;
        imu_gx3.gx3_packet.msg_buf[imu_gx3.gx3_packet.msg_idx] = c;
        imu_gx3.gx3_packet.msg_idx++;
      }
      else {
        imu_gx3.gx3_packet.hdr_error++;
      }
      break;
    case GX3PacketReading:
      imu_gx3.gx3_packet.msg_buf[imu_gx3.gx3_packet.msg_idx] =  c;
      imu_gx3.gx3_packet.msg_idx++;
      if (imu_gx3.gx3_packet.msg_idx == GX3_MSG_LEN) {
        if (gx3_verify_chk(imu_gx3.gx3_packet.msg_buf)) {
          imu_gx3.gx3_packet.msg_available = TRUE;
          imu_gx3.gx3_packet.status = GX3PacketFull;
        }
        else {
          imu_gx3.gx3_packet.msg_available = FALSE;
          imu_gx3.gx3_packet.chksm_error++;
        }
        imu_gx3.gx3_packet.status = GX3PacketWaiting;
      }
      break;
    default:
      imu_gx3.gx3_packet.status = GX3PacketWaiting;
      imu_gx3.gx3_packet.msg_idx = 0;
      break;
  }
}


#define EVT_IMU_TX 4
#define EVT_IMU_RX 8

static EventSource eventImuTx;
static EventSource eventImuRx;

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
  chSysLockFromIsr();
  chEvtBroadcastFlagsI(&eventImuTx, EVT_IMU_TX);
  chSysUnlockFromIsr();
  (void) uartp;
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 *
 * Since we already detect errors on the line with hdr/checksum errors, this
 * is not really needed.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
  if (e & UART_OVERRUN_ERROR) {
    imu_gx3.gx3_packet.hdr_error++;
  }
  if (e & UART_NOISE_ERROR ) {
    imu_gx3.gx3_packet.hdr_error++;
  }
  if (e & UART_FRAMING_ERROR ) {
    imu_gx3.gx3_packet.hdr_error++;
  }
  (void)uartp;
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
  (void) uartp;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
  (void)uartp;
  gx3_packet_parse((uint8_t)c);
  if (imu_gx3.gx3_packet.msg_available) {
    chSysLockFromIsr();
    chEvtBroadcastFlagsI(&eventImuRx, EVT_IMU_RX);
    chSysUnlockFromIsr();
  }
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg_gx3 = {
  NULL,                                                     /* end of tx_buf   */
  txend2,                                                   /* physical end tx */
  rxend,                                                    /* ex buf filled */
  rxchar,                                                   /* char received of RX_IDLE */
  rxerr,                                                    /* rx error       */
  GX3_DMA_BAUD,                                             /*     BITRATE    */
  0,                                                        /*    USART CR1   */
  USART_CR2_STOP1_BITS,                                     /*    USART CR2   */
  0                                                         /*    USART CR3   */
};

/*
 *  IMU RX
 *  Replaces imu_event()
 *  @note: For now GX3 is assumed, no error conditions are checked
 *  since checksum is used and an error on the serial line doesn't
 *  halt the system. This approach saves around 10% CPU
 *  If you want to check error conditions, add event listeners for
 *  serial port and check for SD_ERROR flags
 */
__attribute__((noreturn)) msg_t thd_imu_rx(void *arg)
{
  chRegSetThreadName("pprz_imu_rx");
  (void) arg;
  imu_init();
#if USE_IMU_FLOAT
  imu_float_init();
#endif

  // Init event sources
  chEvtInit(&eventImuTx);
  chEvtInit(&eventImuRx);

  // Init serial port
  uint8_t rxchar[GX3_MSG_LEN] = {0};

  uartStart(&GX3_DMA_PORT, &uart_cfg_gx3);
  uartStopSend(&GX3_DMA_PORT);
  uartStopReceive(&GX3_DMA_PORT);

  // Configuration loop
  EventListener elImuEvtTx;
  chEvtRegister(&eventImuTx, &elImuEvtTx, EVT_IMU_TX);
  eventmask_t imu_eventmask;
  while (imu_gx3.gx3_status != GX3Running) {
    // move the config state machine
    imu_eventmask = chEvtWaitOneTimeout(EVENT_MASK(EVT_IMU_TX), MS2ST(10));
    chEvtGetAndClearFlags(&elImuEvtTx);
    if (imu_eventmask != 0) {
      // transmission complete, move on
      switch (imu_gx3.gx3_status) {
        case GX3StopContinuousMode:
          imu_gx3.gx3_status = GX3StopContinuousMode_OK;
          break;
        case GX3SamplingSettings:
          imu_gx3.gx3_status = GX3SamplingSettings_OK;
          break;
        case GX3ContinuousMode:
          imu_gx3.gx3_status = GX3Running;
          break;
        default:
          break;
      }
    }
    else {
      // restart if not initialized within timeout
      if (imu_gx3.gx3_status != GX3Running) {
        imu_gx3.gx3_status = GX3Uninit;
      }
    }
  }

  // Receive loop
  EventListener elImuEvRx;
  chEvtRegister(&eventImuRx, &elImuEvRx, EVT_IMU_RX);
  uartStartReceive(&GX3_DMA_PORT, sizeof(rxchar), rxchar);
  while (TRUE) {
    chEvtWaitOne(EVENT_MASK(EVT_IMU_RX));
    chEvtGetAndClearFlags(&elImuEvRx);
    if (imu_gx3.gx3_packet.msg_available) {
      chMtxLock(&imu_get_data_flag);
      if (imu_gx3.queue.rear == GX3_QUEUE_SIZE-1) {
        imu_gx3.queue.rear = 0;
      }
      else {
        imu_gx3.queue.rear++;
      }
      if (!((imu_gx3.queue.front == imu_gx3.queue.rear) && (imu_gx3.queue.queue_buf[imu_gx3.queue.front][0] != 0))) {
        memcpy(&imu_gx3.queue.queue_buf[imu_gx3.queue.rear], imu_gx3.gx3_packet.msg_buf, GX3_MSG_LEN);
        if (imu_gx3.queue.length < GX3_QUEUE_SIZE ) {
          imu_gx3.queue.length++;
        }
      }
      chMtxUnlock();
      imu_gx3.gx3_packet.msg_available = FALSE;
    }
  }
}
