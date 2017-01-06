'''
 Copyright (C) 2016 Kason Bennett, Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>

 This file is part of paparazzi.

 paparazzi is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2, or (at your option)
 any later version.

 paparazzi is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with paparazzi; see the file COPYING.  If not, see
 <http://www.gnu.org/licenses/>.

 Sbus fakerator: simulated SBUS radio for HITL testing
'''
import numpy as np

SBUS_NB_CHANNEL = 16
SBUS_BUFF_LENGTH = 22
SBUS_START_BYTE = 0x0f
SBUS_END_BYTE = 0x00
SBUS_BIT_PER_CHANNEL = 11
SBUS_BIT_PER_BYTE = 8

def sbus_decode(buff):
    decoded = [0 for x in range(SBUS_NB_CHANNEL)]
    byte_in_raw_buff = 0
    bit_in_raw_buff = 0
    channel = 0
    bit_in_channel = 0

    for c in range(SBUS_NB_CHANNEL * SBUS_BIT_PER_CHANNEL):
        if buff[byte_in_raw_buff] & (1 << bit_in_raw_buff):
            decoded[channel] |= (1 << bit_in_channel)

        bit_in_raw_buff += 1
        bit_in_channel += 1

        if(bit_in_raw_buff == SBUS_BIT_PER_BYTE):
            bit_in_raw_buff = 0
            byte_in_raw_buff += 1

        if(bit_in_channel == SBUS_BIT_PER_CHANNEL):
            bit_in_channel = 0
            channel += 1

    return decoded



def sbus_encode(buff):
    decoded = [0 for x in range(SBUS_BUFF_LENGTH)]
    decoded[0] = 0
    channel = 0
    cur_byte = 0
    bit_in_byte = 0
    bit_in_channel = 0
    for x in range(SBUS_NB_CHANNEL * SBUS_BIT_PER_CHANNEL):
        if int(buff[channel]) & (1 << bit_in_channel):
            decoded[cur_byte] |= (1 << bit_in_byte)

        bit_in_channel += 1
        bit_in_byte += 1

        if(bit_in_byte == SBUS_BIT_PER_BYTE):
            bit_in_byte = 0
            cur_byte += 1

        if(bit_in_channel == SBUS_BIT_PER_CHANNEL):
            bit_in_channel = 0
            channel += 1

    return [SBUS_START_BYTE] + decoded + [0, SBUS_END_BYTE]


if __name__ == "__main__":
    control = [169, 84, 149, 170, 84, 197, 10, 86, 168, 138, 21, 172, 80, 21, 43, 88, 193, 10, 86, 168, 74, 85, 0, 0]
    print sbus_decode(control)
    print control
    print sbus_encode(sbus_decode(control))

