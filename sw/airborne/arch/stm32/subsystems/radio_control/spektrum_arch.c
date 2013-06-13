/*
 * Copyright (C) 2010 Eric Parsonage <eric@eparsonage.com>
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

#include <stdint.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include "subsystems/radio_control.h"
#include "subsystems/radio_control/spektrum_arch.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/gpio.h"

#include BOARD_CONFIG

#define SPEKTRUM_CHANNELS_PER_FRAME 7
#define MAX_SPEKTRUM_FRAMES 2
#define MAX_SPEKTRUM_CHANNELS 16

#define MAX_DELAY   INT16_MAX
/* the frequency of the delay timer */
#define DELAY_TIM_FREQUENCY 1000000
/* Number of low pulses sent to satellite receivers */
#define MASTER_RECEIVER_PULSES 5
#define SLAVE_RECEIVER_PULSES 6

#define TIM_FREQ_1000000 1000000
#define TIM_TICS_FOR_100us 100
#define MIN_FRAME_SPACE  70  // 7ms
#define MAX_BYTE_SPACE  3   // .3ms

/*
 * in the makefile we set RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT to be UARTx
 * but in uart_hw.c the initialisation functions are
 * defined as uartx these macros give us the glue
 * that allows static calls at compile time
 */

#define __PrimaryUart(dev, _x) dev##_x
#define _PrimaryUart(dev, _x)  __PrimaryUart(dev, _x)
#define PrimaryUart(_x) _PrimaryUart(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT, _x)

#define __SecondaryUart(dev, _x) dev##_x
#define _SecondaryUart(dev, _x)  __SecondaryUart(dev, _x)
#define SecondaryUart(_x) _SecondaryUart(RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT, _x)

struct SpektrumStateStruct {
    uint8_t ReSync;
    uint8_t SpektrumTimer;
    uint8_t Sync;
    uint8_t ChannelCnt;
    uint8_t FrameCnt;
    uint8_t HighByte;
    uint8_t SecondFrame;
    uint16_t LostFrameCnt;
    uint8_t RcAvailable;
    int16_t values[SPEKTRUM_CHANNELS_PER_FRAME*MAX_SPEKTRUM_FRAMES];
};

typedef struct SpektrumStateStruct SpektrumStateType;

SpektrumStateType PrimarySpektrumState = {1,0,0,0,0,0,0,0,0,{0}};
PRINT_CONFIG_VAR(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)

#ifdef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
PRINT_CONFIG_MSG("Using secondary spektrum receiver.")
PRINT_CONFIG_VAR(RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT)
SpektrumStateType SecondarySpektrumState = {1,0,0,0,0,0,0,0,0,{0}};
#else
PRINT_CONFIG_MSG("NOT using secondary spektrum receiver.")
#endif

int16_t SpektrumBuf[SPEKTRUM_CHANNELS_PER_FRAME*MAX_SPEKTRUM_FRAMES];
/* the order of the channels on a spektrum is always as follows :
 *
 * Throttle   0
 * Aileron    1
 * Elevator   2
 * Rudder     3
 * Gear       4
 * Flap/Aux1  5
 * Aux2       6
 * Aux3       7
 * Aux4       8
 * Aux5       9
 * Aux6      10
 * Aux7      11
 */

/* reverse some channels to suit Paparazzi conventions          */
/* the maximum number of channels a Spektrum can transmit is 12 */
int8_t SpektrumSigns[] = RADIO_CONTROL_SPEKTRUM_SIGNS;

/* Parser state variables */
static uint8_t EncodingType = 0;
static uint8_t ExpectedFrames = 0;
/* initialise the uarts used by the parser */
void SpektrumUartInit(void);
/* initialise the timer used by the parser to ensure sync */
void SpektrumTimerInit(void);

void tim6_irq_handler(void);
/* wait busy loop, microseconds */
static void DelayUs( uint16_t uSecs );
/* wait busy loop, milliseconds */
static void DelayMs( uint16_t mSecs );
/* setup timer 1 for busy wait delays */
static void SpektrumDelayInit( void );


 /*****************************************************************************
 *
 * Initialise the timer an uarts used by the Spektrum receiver subsystem
 *
 *****************************************************************************/
void radio_control_impl_init(void) {

  PrimarySpektrumState.ReSync = 1;

#ifdef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
  SecondarySpektrumState.ReSync = 1;
#endif

  SpektrumTimerInit();
  SpektrumUartInit();
}

/*****************************************************************************
 * The bind function means that the satellite receivers believe they are
 * connected to a 9 channel JR-R921 24 receiver thus during the bind process
 * they try to get the transmitter to transmit at the highest resolution that
 * it can manage. The data is contained in 16 byte packets transmitted at
 * 115200 baud. Depending on the transmitter either 1 or 2 frames are required
 * to contain the data for all channels. These frames are either 11ms or 22ms
 * apart.
 *
 * The format of each frame for the main receiver is as follows
 *
 *  byte1:  frame loss data
 *  byte2:  transmitter information
 *  byte3:  and byte4:  channel data
 *  byte5:  and byte6:  channel data
 *  byte7:  and byte8:  channel data
 *  byte9:  and byte10: channel data
 *  byte11: and byte12: channel data
 *  byte13: and byte14: channel data
 *  byte15: and byte16: channel data
 *
 *
 * The format of each frame for the secondary receiver is as follows
 *
 *  byte1:  frame loss data
 *  byte2:  frame loss data
 *  byte3:  and byte4:  channel data
 *  byte5:  and byte6:  channel data
 *  byte7:  and byte8:  channel data
 *  byte9:  and byte10: channel data
 *  byte11: and byte12: channel data
 *  byte13: and byte14: channel data
 *  byte15: and byte16: channel data
 *
 * The frame loss data bytes starts out containing 0 as long as the
 * transmitter is switched on before the receivers. It then increments
 * whenever frames are dropped.
 *
 * Three values for the transmitter information byte have been seen thus far
 *
 * 0x01 From a Spektrum DX7eu which transmits a single frame containing all
 * channel data every 22ms with 10bit resolution.
 *
 * 0x02 From a Spektrum DM9 module which transmits two frames to carry the
 * data for all channels 11ms apart with 10bit resolution.
 *
 * 0x12 From a Spektrum DX7se which transmits two frames to carry the
 * data for all channels 11ms apart with 11bit resolution.
 *
 * 0x12 From a JR X9503 which transmits two frames to carry the
 * data for all channels 11ms apart with 11bit resolution.
 *
 * 0x01 From a Spektrum DX7 which transmits a single frame containing all
 * channel data every 22ms with 10bit resolution.
 *
 * 0x12 From a JR DSX12 which transmits two frames to carry the
 * data for all channels 11ms apart with 11bit resolution.
 *
 * 0x1 From a Spektru DX5e which transmits a single frame containing all
 * channel data every 22ms with 10bit resolution.
 *
 * 0x01 From a Spektrum DX6i which transmits a single frame containing all
 * channel data every 22ms with 10bit resolution.
 *
 * Currently the assumption is that the data has the form :
 *
 * [0 0 0 R 0 0 N1 N0]
 *
 * where :
 *
 * 0 means a '0' bit
 * R: 0 for 10 bit resolution 1 for 11 bit resolution channel data
 * N1 to N0 is the number of frames required to receive all channel
 * data.
 *
 * Channels can have either 10bit or 11bit resolution. Data from a tranmitter
 * with 10 bit resolution has the form:
 *
 * [F 0 C3 C2 C1 C0 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0]
 *
 * Data from a tranmitter with 11 bit resolution has the form
 *
 * [F C3 C2 C1 C0 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0]
 *
 * where :
 *
 * 0 means a '0' bit
 * F: Normally 0 but set to 1 for the first channel of the 2nd frame if a
 * second frame is transmitted.
 *
 * C3 to C0 is the channel number, 4 bit, matching the numbers allocated in
 * the transmitter.
 *
 * D9 to D0 is the channel data (10 bit) 0xaa..0x200..0x356 for
 * 100% transmitter-travel
 *
 *
 * D10 to D0 is the channel data (11 bit) 0x154..0x400..0x6AC for
 * 100% transmitter-travel
 *****************************************************************************/

 /*****************************************************************************
 *
 * Spektrum Parser captures frame data by using time between frames to sync on
 *
 *****************************************************************************/

static inline void SpektrumParser(uint8_t _c, SpektrumStateType* spektrum_state, bool_t secondary_receiver)  {

  uint16_t ChannelData;
  uint8_t TimedOut;
  static uint8_t TmpEncType = 0;        /* 0 = 10bit, 1 = 11 bit        */
  static uint8_t TmpExpFrames = 0;      /* # of frames for channel data */

   TimedOut = (!spektrum_state->SpektrumTimer) ? 1 : 0;

  /* If we have just started the resync process or */
  /* if we have recieved a character before our    */
  /* 7ms wait has finished                         */
  if ((spektrum_state->ReSync == 1) ||
      ((spektrum_state->Sync == 0) && (!TimedOut))) {

    spektrum_state->ReSync = 0;
    spektrum_state->SpektrumTimer = MIN_FRAME_SPACE;
    spektrum_state->Sync = 0;
    spektrum_state->ChannelCnt = 0;
    spektrum_state->FrameCnt = 0;
    spektrum_state->SecondFrame = 0;
    return;
  }

  /* the first byte of a new frame. It was received */
  /* more than 7ms after the last received byte.    */
  /* It represents the number of lost frames so far.*/
  if (spektrum_state->Sync == 0) {
      spektrum_state->LostFrameCnt = _c;
      if(secondary_receiver) /* secondary receiver */
        spektrum_state->LostFrameCnt = spektrum_state->LostFrameCnt << 8;
      spektrum_state->Sync = 1;
      spektrum_state->SpektrumTimer = MAX_BYTE_SPACE;
      return;
  }

  /* all other bytes should be recieved within     */
  /* MAX_BYTE_SPACE time of the last byte received */
  /* otherwise something went wrong resynchronise  */
  if(TimedOut) {
    spektrum_state->ReSync = 1;
    /* next frame not expected sooner than 7ms     */
    spektrum_state->SpektrumTimer = MIN_FRAME_SPACE;
    return;
  }

  /* second character determines resolution and frame rate for main */
  /* receiver or low byte of LostFrameCount for secondary receiver  */
  if(spektrum_state->Sync == 1) {
    if(secondary_receiver) {
      spektrum_state->LostFrameCnt +=_c;
      TmpExpFrames = ExpectedFrames;
    } else {
      /** @todo collect more data. I suspect that there is a low res       */
      /* protocol that is still 10 bit but without using the full range.    */
      TmpEncType =(_c & 0x10)>>4;      /* 0 = 10bit, 1 = 11 bit             */
      TmpExpFrames = _c & 0x03;        /* 1 = 1 frame contains all channels */
                                       /* 2 = 2 channel data in 2 frames    */
    }
    spektrum_state->Sync = 2;
    spektrum_state->SpektrumTimer = MAX_BYTE_SPACE;
    return;
  }

  /* high byte of channel data if this is the first byte */
  /* of channel data and the most significant bit is set */
  /* then this is the second frame of channel data.      */
  if(spektrum_state->Sync == 2) {
    spektrum_state->HighByte = _c;
    if (spektrum_state->ChannelCnt == 0) {
      spektrum_state->SecondFrame = (spektrum_state->HighByte & 0x80) ? 1 : 0;
    }
    spektrum_state->Sync = 3;
    spektrum_state->SpektrumTimer = MAX_BYTE_SPACE;
    return;
  }

  /* low byte of channel data */
  if(spektrum_state->Sync == 3) {
    spektrum_state->Sync = 2;
    spektrum_state->SpektrumTimer = MAX_BYTE_SPACE;
    /* we overwrite the buffer now so rc data is not available now */
    spektrum_state->RcAvailable = 0;
    ChannelData = ((uint16_t)spektrum_state->HighByte << 8) | _c;
    spektrum_state->values[spektrum_state->ChannelCnt
                          + (spektrum_state->SecondFrame * 7)] = ChannelData;
    spektrum_state->ChannelCnt ++;
  }

  /* If we have a whole frame */
  if(spektrum_state->ChannelCnt >= SPEKTRUM_CHANNELS_PER_FRAME) {
    /* how many frames did we expect ? */
    ++spektrum_state->FrameCnt;
    if (spektrum_state->FrameCnt == TmpExpFrames)
    {
      /* set the rc_available_flag */
      spektrum_state->RcAvailable = 1;
      spektrum_state->FrameCnt = 0;
    }
    if(!secondary_receiver) { /* main receiver */
      EncodingType = TmpEncType;         /* only update on a good */
      ExpectedFrames = TmpExpFrames;     /* main receiver frame   */
    }
    spektrum_state->Sync = 0;
    spektrum_state->ChannelCnt = 0;
    spektrum_state->SecondFrame = 0;
    spektrum_state->SpektrumTimer = MIN_FRAME_SPACE;
  }
}

/*****************************************************************************
 *
 * RadioControlEventImp decodes channel data stored by uart irq handlers
 * and calls callback funtion
 *
 *****************************************************************************/

void RadioControlEventImp(void (*frame_handler)(void)) {
  uint8_t ChannelCnt;
  uint8_t ChannelNum;
  uint16_t ChannelData;
  uint8_t MaxChannelNum = 0;

#ifdef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
  /* If we have two receivers and at least one of them has new data */
  uint8_t BestReceiver;
  if ((PrimarySpektrumState.RcAvailable) ||
      (SecondarySpektrumState.RcAvailable)) {
    /* if both receivers have new data select the one  */
    /* that has had the least number of frames lost    */
    if ((PrimarySpektrumState.RcAvailable) &&
        (SecondarySpektrumState.RcAvailable)) {
      BestReceiver  = (PrimarySpektrumState.LostFrameCnt
                       <= SecondarySpektrumState.LostFrameCnt) ? 0 : 1;
    } else {
      /* if only one of the receivers have new data use it */
      BestReceiver  = (PrimarySpektrumState.RcAvailable) ? 0 : 1;
    }
    /* clear the data ready flags */
    PrimarySpektrumState.RcAvailable = 0;
    SecondarySpektrumState.RcAvailable = 0;

#else
  /* if we have one receiver and it has new data */
  if(PrimarySpektrumState.RcAvailable) {
    PrimarySpektrumState.RcAvailable = 0;
#endif
    ChannelCnt = 0;
    /* for every piece of channel data we have received */
    for(int i = 0; (i < SPEKTRUM_CHANNELS_PER_FRAME * ExpectedFrames); i++) {
#ifndef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
      ChannelData = PrimarySpektrumState.values[i];
#else
      ChannelData = (!BestReceiver) ? PrimarySpektrumState.values[i] :
                                 SecondarySpektrumState.values[i];
#endif
      /* find out the channel number and its value by  */
      /* using the EncodingType which is only received */
      /* from the main receiver                        */
      switch(EncodingType) {
        case(0) : /* 10 bit */
          ChannelNum = (ChannelData >> 10) & 0x0f;
          /* don't bother decoding unused channels */
          if (ChannelNum < RADIO_CONTROL_NB_CHANNEL) {
           SpektrumBuf[ChannelNum] = ChannelData & 0x3ff;
           SpektrumBuf[ChannelNum] -= 0x200;
           SpektrumBuf[ChannelNum] *= MAX_PPRZ/0x156;
           ChannelCnt++;
          }
          break;

        case(1) : /* 11 bit */
          ChannelNum = (ChannelData >> 11) & 0x0f;
          /* don't bother decoding unused channels */
          if (ChannelNum < RADIO_CONTROL_NB_CHANNEL) {
            SpektrumBuf[ChannelNum] = ChannelData & 0x7ff;
            SpektrumBuf[ChannelNum] -= 0x400;
            SpektrumBuf[ChannelNum] *= MAX_PPRZ/0x2AC;
            ChannelCnt++;
          }
          break;

        default : ChannelNum = 0x0F; break;  /* never going to get here */
      }
      /* store the value of the highest valid channel */
      if ((ChannelNum != 0x0F) && (ChannelNum > MaxChannelNum))
        MaxChannelNum = ChannelNum;

    }

    /* if we have a valid frame the pass it to the frame handler */
    if (ChannelCnt >= (MaxChannelNum + 1)) {
      radio_control.frame_cpt++;
      radio_control.time_since_last_frame = 0;
      radio_control.status = RC_OK;
      for (int i = 0; i < (MaxChannelNum + 1); i++) {
        radio_control.values[i] = SpektrumBuf[i];
        if (i == RADIO_THROTTLE ) {
          radio_control.values[i] += MAX_PPRZ;
          radio_control.values[i] /= 2;
        }
        radio_control.values[i] *= SpektrumSigns[i];
      }
      (*frame_handler)();
    }
  }
}


/*****************************************************************************
 *
 * Initialise TIM6 to fire an interrupt every 100 microseconds to provide
 * timebase for SpektrumParser
 *
 *****************************************************************************/
void SpektrumTimerInit( void ) {

  /* enable TIM6 clock */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM6EN);

  /* TIM6 configuration */
  timer_set_mode(TIM6, TIM_CR1_CKD_CK_INT,
             TIM_CR1_CMS_EDGE, TIM_CR1_DIR_DOWN);
  /* 100 microseconds ie 0.1 millisecond */
  timer_set_period(TIM6, TIM_TICS_FOR_100us-1);
  timer_set_prescaler(TIM6, ((AHB_CLK / TIM_FREQ_1000000) - 1));

  /* Enable TIM6 interrupts */
#ifdef STM32F1
  nvic_set_priority(NVIC_TIM6_IRQ, 2);
  nvic_enable_irq(NVIC_TIM6_IRQ);
#elif defined STM32F4
  /* the define says DAC IRQ, but it is also the global TIM6 IRQ*/
  nvic_set_priority(NVIC_TIM6_DAC_IRQ, 2);
  nvic_enable_irq(NVIC_TIM6_DAC_IRQ);
#endif

  /* Enable TIM6 Update interrupt */
  timer_enable_irq(TIM6, TIM_DIER_UIE);
  timer_clear_flag(TIM6, TIM_SR_UIF);

  /* TIM6 enable counter */
  timer_enable_counter(TIM6);
}

/*****************************************************************************
 *
 * TIM6 interrupt request handler updates times used by SpektrumParser
 *
 *****************************************************************************/
#ifdef STM32F1
void tim6_isr( void ) {
#elif defined STM32F4
void tim6_dac_isr( void ) {
#endif

  timer_clear_flag(TIM6, TIM_SR_UIF);

  if (PrimarySpektrumState.SpektrumTimer)
    --PrimarySpektrumState.SpektrumTimer;
#ifdef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
  if (SecondarySpektrumState.SpektrumTimer)
    --SecondarySpektrumState.SpektrumTimer;
#endif
}

/*****************************************************************************
 *
 * Initialise the uarts for the spektrum satellite receivers
 *
 *****************************************************************************/
void SpektrumUartInit(void) {
  /* init RCC */
  gpio_enable_clock(PrimaryUart(_BANK));
  rcc_peripheral_enable_clock(PrimaryUart(_RCC_REG), PrimaryUart(_RCC_DEV));

  /* Enable USART interrupts */
  nvic_set_priority(PrimaryUart(_IRQ), 2);
  nvic_enable_irq(PrimaryUart(_IRQ));

  /* Init GPIOS */
  /* Primary UART Rx pin as floating input */
  gpio_setup_pin_af(PrimaryUart(_BANK), PrimaryUart(_PIN), PrimaryUart(_AF), FALSE);

  /* Configure Primary UART */
  usart_set_baudrate(PrimaryUart(_DEV), 115200);
  usart_set_databits(PrimaryUart(_DEV), 8);
  usart_set_stopbits(PrimaryUart(_DEV), USART_STOPBITS_1);
  usart_set_parity(PrimaryUart(_DEV), USART_PARITY_NONE);
  usart_set_flow_control(PrimaryUart(_DEV), USART_FLOWCONTROL_NONE);
  usart_set_mode(PrimaryUart(_DEV), USART_MODE_RX);

  /* Enable Primary UART Receive interrupts */
  USART_CR1(PrimaryUart(_DEV)) |= USART_CR1_RXNEIE;

  /* Enable the Primary UART */
  usart_enable(PrimaryUart(_DEV));

#ifdef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
  /* init RCC */
  gpio_enable_clock(SecondaryUart(_BANK));
  rcc_peripheral_enable_clock(SecondaryUart(_RCC_REG), SecondaryUart(_RCC_DEV));

  /* Enable USART interrupts */
  nvic_set_priority(SecondaryUart(_IRQ), 3);
  nvic_enable_irq(SecondaryUart(_IRQ));

  /* Init GPIOS */;
  /* Secondary UART Rx pin as floating input */
  gpio_setup_pin_af(SecondaryUart(_BANK), SecondaryUart(_PIN), SecondaryUart(_AF), FALSE);

  /* Configure secondary UART */
  usart_set_baudrate(SecondaryUart(_DEV), 115200);
  usart_set_databits(SecondaryUart(_DEV), 8);
  usart_set_stopbits(SecondaryUart(_DEV), USART_STOPBITS_1);
  usart_set_parity(SecondaryUart(_DEV), USART_PARITY_NONE);
  usart_set_flow_control(SecondaryUart(_DEV), USART_FLOWCONTROL_NONE);
  usart_set_mode(SecondaryUart(_DEV), USART_MODE_RX);

  /* Enable Secondary UART Receive interrupts */
  USART_CR1(SecondaryUart(_DEV)) |= USART_CR1_RXNEIE;

  /* Enable the Primary UART */
  usart_enable(SecondaryUart(_DEV));
#endif

}

/*****************************************************************************
 *
 * The primary receiver UART interrupt request handler which passes the
 * received character to Spektrum Parser.
 *
 *****************************************************************************/
void PrimaryUart(_ISR)(void) {

  if (((USART_CR1(PrimaryUart(_DEV)) & USART_CR1_TXEIE) != 0) &&
      ((USART_SR(PrimaryUart(_DEV)) & USART_SR_TXE) != 0)) {
    USART_CR1(PrimaryUart(_DEV)) &= ~USART_CR1_TXEIE;
  }

  if (((USART_CR1(PrimaryUart(_DEV)) & USART_CR1_RXNEIE) != 0) &&
      ((USART_SR(PrimaryUart(_DEV)) & USART_SR_RXNE) != 0)) {
    uint8_t b = usart_recv(PrimaryUart(_DEV));
    SpektrumParser(b, &PrimarySpektrumState, FALSE);
  }

}

/*****************************************************************************
 *
 * The secondary receiver UART interrupt request handler which passes the
 * received character to Spektrum Parser.
 *
 *****************************************************************************/
#ifdef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
void SecondaryUart(_ISR)(void) {

  if (((USART_CR1(SecondaryUart(_DEV)) & USART_CR1_TXEIE) != 0) &&
      ((USART_SR(SecondaryUart(_DEV)) & USART_SR_TXE) != 0)) {
    USART_CR1(SecondaryUart(_DEV)) &= ~USART_CR1_TXEIE;
  }

  if (((USART_CR1(SecondaryUart(_DEV)) & USART_CR1_RXNEIE) != 0) &&
      ((USART_SR(SecondaryUart(_DEV)) & USART_SR_RXNE) != 0)) {
    uint8_t b = usart_recv(SecondaryUart(_DEV));
    SpektrumParser(b, &SecondarySpektrumState, TRUE);
  }

}
#endif


/*****************************************************************************
 *
 * The following functions provide functionality to allow binding of
 * spektrum satellite receivers. The pulse train sent to them means
 * that Lisa is emulating a 9 channel JR-R921 24.
 *
 *****************************************************************************/
/*****************************************************************************
 *
 * radio_control_spektrum_try_bind(void) must called on powerup as spektrum
 * satellites can only bind immediately after power up also it must be called
 * before the call to SpektrumUartInit as we leave them with their Rx pins set
 * as outputs.
 *
 *****************************************************************************/
void radio_control_spektrum_try_bind(void) {

  /* Init GPIO for the bind pin */
  gpio_setup_input(SPEKTRUM_BIND_PIN_PORT, GPIO_MODE_INPUT);

  /* exit if the BIND_PIN is high, it needs to
     be pulled low at startup to initiate bind */
  if (gpio_get(SPEKTRUM_BIND_PIN_PORT, SPEKTRUM_BIND_PIN) != 0)
    return;

  /* bind initiated, initialise the delay timer */
  SpektrumDelayInit();

  /* initialise the uarts rx pins as  GPIOS */
  gpio_enable_clock(PrimaryUart(_BANK));

  /* Master receiver Rx push-pull */
  gpio_setup_output(PrimaryUart(_BANK), PrimaryUart(_PIN));

  /* Master receiver RX line, drive high */
  gpio_set(PrimaryUart(_BANK), PrimaryUart(_PIN));

#ifdef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT

  gpio_enable_clock(SecondaryUart(_BANK));

  /* Slave receiver Rx push-pull */
  gpio_setup_output(SecondaryUart(_BANK), SecondaryUart(_PIN));

  /* Slave receiver RX line, drive high */
  gpio_set(SecondaryUart(_BANK), SecondaryUart(_PIN));
#endif

  /* We have no idea how long the window for allowing binding after
     power up is. This works for the moment but will need revisiting */
  DelayMs(61);

  for (int i = 0; i < MASTER_RECEIVER_PULSES ; i++)
  {
    gpio_clear(PrimaryUart(_BANK), PrimaryUart(_PIN));
    DelayUs(118);
    gpio_set(PrimaryUart(_BANK), PrimaryUart(_PIN));
    DelayUs(122);
  }

#ifdef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
  for (int i = 0; i < SLAVE_RECEIVER_PULSES; i++)
  {
    gpio_clear(SecondaryUart(_BANK), SecondaryUart(_PIN));
    DelayUs(120);
    gpio_set(SecondaryUart(_BANK), SecondaryUart(_PIN));
    DelayUs(120);
  }
#endif /* RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT */
}

/*****************************************************************************
 *
 * Functions to implement busy wait loops with micro second granularity
 *
 *****************************************************************************/

/* set TIM6 to run at DELAY_TIM_FREQUENCY */
static void SpektrumDelayInit( void ) {

  /* Enable timer clock */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM6EN);

  /* Make sure the timer is reset to default values. */
  timer_reset(TIM6);

  /* Time base configuration */
  /* Mode does not need to be set as the default reset values are ok. */
  timer_set_period(TIM6, UINT16_MAX);
  timer_set_prescaler(TIM6, (AHB_CLK / DELAY_TIM_FREQUENCY) - 1);

  /*
   * Let's start the timer late in the cycle to force an update event before
   * we start using this timer for generating delays. Otherwise the prescaler
   * value does not seem to be taken over by the timer, resulting in way too
   * high counting frequency. There does not seem to be a force update bit on
   * TIM6 is there?
   */
  TIM6_CNT = 65534;

  /* Enable counter */
  timer_enable_counter(TIM6);
}

/* wait busy loop, microseconds */
static void DelayUs( uint16_t uSecs ) {
  uint16_t start = TIM6_CNT;

  /* use 16 bit count wrap around */
  while((TIM6_CNT - start) <= uSecs);
}

/* wait busy loop, milliseconds */
static void DelayMs( uint16_t mSecs ) {

  for(int i = 0; i < mSecs; i++) {
    DelayUs(DELAY_TIM_FREQUENCY / 1000);
  }
}
