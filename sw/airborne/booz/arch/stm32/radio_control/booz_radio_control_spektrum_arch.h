/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2009-2010 The Paparazzi Team
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

#ifndef BOOZ_RADIO_CONTROL_SPEKTRUM_ARCH_H
#define BOOZ_RADIO_CONTROL_SPEKTRUM_ARCH_H

#include "std.h"
#include "uart.h"

#include RADIO_CONTROL_SPEKTRUM_MODEL_H

#define SPEKTRUM_CHANNELS_PER_FRAME 7
#define MAX_SPEKTRUM_FRAMES 2
#define MAX_SPEKTRUM_CHANNELS 16

extern int16_t SpektrumBuf[SPEKTRUM_CHANNELS_PER_FRAME*MAX_SPEKTRUM_FRAMES];
extern int8_t SpektrumSigns[MAX_SPEKTRUM_CHANNELS];

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

extern SpektrumStateType PrimarySpektrumState;
#ifdef RADIO_CONTROL_LINK_SLAVE 
extern SpektrumStateType SecondarySpektrumState;
#endif


extern void _RadioControlEventImp(void (*_received_frame_handler)(void));

#define RadioControlEventImp(_received_frame_handler) _RadioControlEventImp(_received_frame_handler);



#endif /* BOOZ_RADIO_CONTROL_SPEKTRUM_ARCH_H */
