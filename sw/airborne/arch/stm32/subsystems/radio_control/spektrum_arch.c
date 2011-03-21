/*
 * $Id$
 *
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
#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include <stm32/tim.h>
#include <stm32/misc.h>
#include <stm32/usart.h>
#include "subsystems/radio_control.h"
#include "subsystems/radio_control/spektrum_arch.h"
#include "mcu_periph/uart.h"
#include "pprz_baudrate.h"



#define SPEKTRUM_CHANNELS_PER_FRAME 7
#define MAX_SPEKTRUM_FRAMES 2
#define MAX_SPEKTRUM_CHANNELS 16

#define MAX_DELAY   INT16_MAX
/* the frequency of the delay timer */
#define DELAY_TIM_FREQUENCY 1000000
/* Number of low pulses sent to satellite receivers */
#define MASTER_RECEIVER_PULSES 5
#define SLAVE_RECEIVER_PULSES 6

/* The line that is pulled low at power up to initiate the bind process */
#define BIND_PIN GPIO_Pin_3
#define BIND_PIN_PORT GPIOC
#define BIND_PIN_PERIPH RCC_APB2Periph_GPIOC

#define TIM_FREQ_1000000 1000000
#define TIM_TICS_FOR_100us 100
#define MIN_FRAME_SPACE  70  // 7ms
#define MAX_BYTE_SPACE  3   // .3ms

/*
 * in the makefile we set RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT to be Uartx
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

SpektrumStateType PrimarySpektrumState = {1,0,0,0,0,0,0,0,0};
#ifdef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
#warning "Using secondary spektrum receiver."
SpektrumStateType SecondarySpektrumState = {1,0,0,0,0,0,0,0,0};
#else
#warning "NOT using secondary spektrum receiver."
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
/* sets a GPIO pin as output for debugging */
void DebugInit(void);
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
  SpektrumTimerInit();
  // DebugInit();
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

#define SpektrumParser(_c, _SpektrumState, _receiver)  {                      \
                                                                              \
  uint16_t ChannelData;                                                       \
  uint8_t TimedOut;                                                           \
  static uint8_t TmpEncType = 0;        /* 0 = 10bit, 1 = 11 bit        */    \
  static uint8_t TmpExpFrames = 0;      /* # of frames for channel data */    \
                                                                              \
   TimedOut = (!_SpektrumState.SpektrumTimer) ? 1 : 0;                        \
                                                                              \
  /* If we have just started the resync process or */                         \
  /* if we have recieved a character before our    */                         \
  /* 7ms wait has finished                         */                         \
  if ((_SpektrumState.ReSync == 1) ||                                         \
      ((_SpektrumState.Sync == 0) && (!TimedOut))) {                          \
                                                                              \
    _SpektrumState.ReSync = 0;                                                \
    _SpektrumState.SpektrumTimer = MIN_FRAME_SPACE;                           \
    _SpektrumState.Sync = 0;                                                  \
    _SpektrumState.ChannelCnt = 0;                                            \
    _SpektrumState.FrameCnt = 0;                                              \
    _SpektrumState.SecondFrame = 0;                                           \
    return;                                                                   \
  }                                                                           \
                                                                              \
  /* the first byte of a new frame. It was received */                        \
  /* more than 7ms after the last received byte.    */                        \
  /* It represents the number of lost frames so far.*/                        \
  if (_SpektrumState.Sync == 0) {                                             \
      _SpektrumState.LostFrameCnt = _c;                                       \
      if(_receiver) /* secondary receiver */                                  \
        _SpektrumState.LostFrameCnt = _SpektrumState.LostFrameCnt << 8;       \
      _SpektrumState.Sync = 1;                                                \
      _SpektrumState.SpektrumTimer = MAX_BYTE_SPACE;                          \
      return;                                                                 \
  }                                                                           \
                                                                              \
  /* all other bytes should be recieved within     */                         \
  /* MAX_BYTE_SPACE time of the last byte received */                         \
  /* otherwise something went wrong resynchronise  */                         \
  if(TimedOut) {                                                              \
    _SpektrumState.ReSync = 1;                                                \
    /* next frame not expected sooner than 7ms     */                         \
    _SpektrumState.SpektrumTimer = MIN_FRAME_SPACE;                           \
    return;                                                                   \
  }                                                                           \
                                                                              \
  /* second character determines resolution and frame rate for main */        \
  /* receiver or low byte of LostFrameCount for secondary receiver  */        \
  if(_SpektrumState.Sync == 1) {                                              \
    if(_receiver) {                                                           \
      _SpektrumState.LostFrameCnt +=_c;                                       \
      TmpExpFrames = ExpectedFrames;                                          \
    } else {                                                                  \
      /* TODO: collect more data. I suspect that there is a low res         */\
      /* protocol that is still 10 bit but without using the full range.    */\
      TmpEncType =(_c & 0x10)>>4;      /* 0 = 10bit, 1 = 11 bit             */\
      TmpExpFrames = _c & 0x03;        /* 1 = 1 frame contains all channels */\
                                       /* 2 = 2 channel data in 2 frames    */\
    }                                                                         \
    _SpektrumState.Sync = 2;                                                  \
    _SpektrumState.SpektrumTimer = MAX_BYTE_SPACE;                            \
    return;                                                                   \
  }                                                                           \
                                                                              \
  /* high byte of channel data if this is the first byte */                   \
  /* of channel data and the most significant bit is set */                   \
  /* then this is the second frame of channel data.      */                   \
  if(_SpektrumState.Sync == 2) {                                              \
    _SpektrumState.HighByte = _c;                                             \
    if (_SpektrumState.ChannelCnt == 0) {                                     \
      _SpektrumState.SecondFrame = (_SpektrumState.HighByte & 0x80) ? 1 : 0;  \
    }                                                                         \
    _SpektrumState.Sync = 3;                                                  \
    _SpektrumState.SpektrumTimer = MAX_BYTE_SPACE;                            \
    return;                                                                   \
  }                                                                           \
                                                                              \
  /* low byte of channel data */                                              \
  if(_SpektrumState.Sync == 3) {                                              \
    _SpektrumState.Sync = 2;                                                  \
    _SpektrumState.SpektrumTimer = MAX_BYTE_SPACE;                            \
    /* we overwrite the buffer now so rc data is not available now */         \
    _SpektrumState.RcAvailable = 0;                                           \
    ChannelData = ((uint16_t)_SpektrumState.HighByte << 8) | _c;              \
    _SpektrumState.values[_SpektrumState.ChannelCnt                           \
                          + (_SpektrumState.SecondFrame * 7)] = ChannelData;  \
    _SpektrumState.ChannelCnt ++;                                             \
  }                                                                           \
                                                                              \
  /* If we have a whole frame */                                              \
  if(_SpektrumState.ChannelCnt >= SPEKTRUM_CHANNELS_PER_FRAME) {              \
    /* how many frames did we expect ? */                                     \
    ++_SpektrumState.FrameCnt;                                                \
    if (_SpektrumState.FrameCnt == TmpExpFrames)                              \
    {                                                                         \
      /* set the rc_available_flag */                                         \
      _SpektrumState.RcAvailable = 1;                                         \
      _SpektrumState.FrameCnt = 0;                                            \
    }                                                                         \
    if(!_receiver) { /* main receiver */                                      \
      EncodingType = TmpEncType;         /* only update on a good */          \
      ExpectedFrames = TmpExpFrames;     /* main receiver frame   */          \
    }                                                                         \
    _SpektrumState.Sync = 0;                                                  \
    _SpektrumState.ChannelCnt = 0;                                            \
    _SpektrumState.SecondFrame = 0;                                           \
    _SpektrumState.SpektrumTimer = MIN_FRAME_SPACE;                           \
  }                                                                           \
}                                                                             \

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
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  /* TIM6 configuration */
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  /* 100 microseconds ie 0.1 millisecond */
  TIM_TimeBaseStructure.TIM_Period = TIM_TICS_FOR_100us-1;
  TIM_TimeBaseStructure.TIM_Prescaler = ((AHB_CLK / TIM_FREQ_1000000) - 1);
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

  /* Enable TIM6 interrupts */
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable and configure TIM6 IRQ channel */
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable TIM6 Update interrupt */
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
  TIM_ClearFlag(TIM6, TIM_FLAG_Update);

  /* TIM6 enable counter */
  TIM_Cmd(TIM6, ENABLE);
}

/*****************************************************************************
 *
 * TIM6 interrupt request handler updates times used by SpektrumParser
 *
 *****************************************************************************/
void tim6_irq_handler( void ) {

  TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

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
  PrimaryUart(_remap);
  PrimaryUart(_clk)(PrimaryUart(_UartPeriph), ENABLE);;
  //RCC_APB1PeriphClockCmd(PrimaryUart(_UartPeriph), ENABLE);

  /* Enable USART interrupts */
  NVIC_InitTypeDef nvic;
  nvic.NVIC_IRQChannel = PrimaryUart(_IRQn);
  nvic.NVIC_IRQChannelPreemptionPriority = 2;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
  /* Init GPIOS */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  /* Primary UART Rx pin as floating input */
  GPIO_InitStructure.GPIO_Pin   = PrimaryUart(_RxPin);
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(PrimaryUart(_RxPort), &GPIO_InitStructure);
  /* Configure Primary UART */
  USART_InitTypeDef usart;
  usart.USART_BaudRate            = B115200;
  usart.USART_WordLength          = USART_WordLength_8b;
  usart.USART_StopBits            = USART_StopBits_1;
  usart.USART_Parity              = USART_Parity_No;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart.USART_Mode                = USART_Mode_Rx;
  USART_Init(PrimaryUart(_reg), &usart);
  /* Enable Primary UART Receive interrupts */
  USART_ITConfig(PrimaryUart(_reg), USART_IT_RXNE, ENABLE);
 
  /* required to get the correct baudrate on lisa m */   
  pprz_usart_set_baudrate(PrimaryUart(_reg), B115200);
  /* Enable the Primary UART */
  USART_Cmd(PrimaryUart(_reg), ENABLE);


#ifdef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
   /* init RCC */
  SecondaryUart(_remap);
  SecondaryUart(_clk)(SecondaryUart(_UartPeriph), ENABLE);
  //RCC_APB1PeriphClockCmd(SecondaryUart(_UartPeriph), ENABLE);
  /* Enable USART interrupts */
  nvic.NVIC_IRQChannel = SecondaryUart(_IRQn);
  nvic.NVIC_IRQChannelPreemptionPriority = 2;
  nvic.NVIC_IRQChannelSubPriority = 2;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
  /* Init GPIOS */;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  /* Secondary UART Rx pin as floating input */
  GPIO_InitStructure.GPIO_Pin   = SecondaryUart(_RxPin);
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(SecondaryUart(_RxPort), &GPIO_InitStructure);
  /* Configure secondary UART */
  usart.USART_BaudRate            = B115200;
  usart.USART_WordLength          = USART_WordLength_8b;
  usart.USART_StopBits            = USART_StopBits_1;
  usart.USART_Parity              = USART_Parity_No;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart.USART_Mode                = USART_Mode_Rx;
  USART_Init(SecondaryUart(_reg), &usart);
  /* Enable Secondary UART Receive interrupts */
  USART_ITConfig(SecondaryUart(_reg), USART_IT_RXNE, ENABLE);
  
  /* required to get the correct baudrate on lisa m */  
  pprz_usart_set_baudrate(SecondaryUart(_reg), B115200);  
  /* Enable the Primary UART */
  USART_Cmd(SecondaryUart(_reg), ENABLE);
#endif

}

/*****************************************************************************
 *
 * The primary receiver UART interrupt request handler which passes the
 * received character to Spektrum Parser.
 *
 *****************************************************************************/
void PrimaryUart(_irq_handler)(void) {

  if(USART_GetITStatus(PrimaryUart(_reg), USART_IT_TXE) != RESET) {
      USART_ITConfig(PrimaryUart(_reg), USART_IT_TXE, DISABLE);
  }

  if(USART_GetITStatus(PrimaryUart(_reg), USART_IT_RXNE) != RESET) {
    uint8_t b =  USART_ReceiveData(PrimaryUart(_reg));
    SpektrumParser(b, PrimarySpektrumState, 0);
  }
}

/*****************************************************************************
 *
 * The secondary receiver UART interrupt request handler which passes the
 * received character to Spektrum Parser.
 *
 *****************************************************************************/
#ifdef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
void SecondaryUart(_irq_handler)(void) {

  if(USART_GetITStatus(SecondaryUart(_reg), USART_IT_TXE) != RESET) {
      USART_ITConfig(SecondaryUart(_reg), USART_IT_TXE, DISABLE);
  }

  if(USART_GetITStatus(SecondaryUart(_reg), USART_IT_RXNE) != RESET) {
    uint8_t b =  USART_ReceiveData(SecondaryUart(_reg));
    SpektrumParser(b, SecondarySpektrumState, 1);
  }
}
#endif

/*****************************************************************************
 *
 * Use pin to output debug information.
 *
 *****************************************************************************/
void DebugInit(void) {
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOC, GPIO_Pin_5 , Bit_RESET );
}

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

  /* init RCC */
  RCC_APB2PeriphClockCmd(BIND_PIN_PERIPH , ENABLE);

  /* Init GPIO for the bind pin */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = BIND_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(BIND_PIN_PORT, &GPIO_InitStructure);
  /* exit if the BIND_PIN is high, it needs to
     be pulled low at startup to initiate bind */
  if (GPIO_ReadInputDataBit(BIND_PIN_PORT, BIND_PIN))
    return;

  /* bind initiated, initialise the delay timer */
  SpektrumDelayInit();

  /* initialise the uarts rx pins as  GPIOS */
  RCC_APB2PeriphClockCmd(PrimaryUart(_Periph) , ENABLE);
  /* Master receiver Rx push-pull */
  GPIO_InitStructure.GPIO_Pin = PrimaryUart(_RxPin);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(PrimaryUart(_RxPort), &GPIO_InitStructure);
  /* Master receiver RX line, drive high */
  GPIO_WriteBit(PrimaryUart(_RxPort), PrimaryUart(_RxPin) , Bit_SET );

#ifdef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
   RCC_APB2PeriphClockCmd(SecondaryUart(_Periph) , ENABLE);
  /* Slave receiver Rx push-pull */
  GPIO_InitStructure.GPIO_Pin = SecondaryUart(_RxPin);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SecondaryUart(_RxPort), &GPIO_InitStructure);
  /* Slave receiver RX line, drive high */
  GPIO_WriteBit(SecondaryUart(_RxPort), SecondaryUart(_RxPin) , Bit_SET );
#endif

  /* We have no idea how long the window for allowing binding after
     power up is. This works for the moment but will need revisiting */
  DelayMs(61);

  for (int i = 0; i < MASTER_RECEIVER_PULSES ; i++)
  {
    GPIO_WriteBit(PrimaryUart(_RxPort), PrimaryUart(_RxPin), Bit_RESET );
    DelayUs(118);
    GPIO_WriteBit(PrimaryUart(_RxPort), PrimaryUart(_RxPin), Bit_SET );
    DelayUs(122);
  }

#ifdef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
  for (int i = 0; i < SLAVE_RECEIVER_PULSES; i++)
  {
    GPIO_WriteBit(SecondaryUart(_RxPort), SecondaryUart(_RxPin), Bit_RESET );
    DelayUs(120);
    GPIO_WriteBit(SecondaryUart(_RxPort), SecondaryUart(_RxPin), Bit_SET );
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
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  /* Time base configuration */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = (AHB_CLK / DELAY_TIM_FREQUENCY) - 1;
  TIM_TimeBaseStructure.TIM_Period = UINT16_MAX;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

 /* Enable counter */
  TIM_Cmd(TIM6, ENABLE);
}

/* wait busy loop, microseconds */
static void DelayUs( uint16_t uSecs ) {
  uint16_t start = TIM6->CNT;
  /* use 16 bit count wrap around */
  while((uint16_t)(TIM6->CNT - start) <= uSecs);
}

/* wait busy loop, milliseconds */
static void DelayMs( uint16_t mSecs ) {
  for(int i = 0; i < mSecs; i++) {
    DelayUs(DELAY_TIM_FREQUENCY / 1000);
  }
}
