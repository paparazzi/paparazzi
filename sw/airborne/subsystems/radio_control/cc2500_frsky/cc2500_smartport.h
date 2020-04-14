#include "cc2500_compat.h"

/* SmartPort downlink hook
 * Called from smartport telemetry loop.
 * Write telemetry stream to 'data'.
 * Return 'true' if data is written.
 */
typedef bool smartPortDownlinkFn(uint32_t *data);
extern smartPortDownlinkFn *smartPortDownlink;

/* SmartPort uplink hook
 * Called from processSmartPortTelemetry
 */
struct smartPortPayload_s;
typedef void smartPortUplinkFn(struct smartPortPayload_s *payload);
extern smartPortUplinkFn *smartPortUplink;

// CAUTION: LARGE PARTS OF THIS FILE ARE COMMENTED OUT!
// betaflight/src/main/telemetry/smartport.h @ 41492e1
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * smartport.h
 *
 *  Created on: 25 October 2014
 *      Author: Frank26080115
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

//#define SMARTPORT_MSP_TX_BUF_SIZE 256
//#define SMARTPORT_MSP_RX_BUF_SIZE 64

enum
{
    FSSP_START_STOP = 0x7E,

    FSSP_DLE        = 0x7D,
    FSSP_DLE_XOR    = 0x20,

    FSSP_DATA_FRAME = 0x10,
    FSSP_MSPC_FRAME_SMARTPORT = 0x30, // MSP client frame
    FSSP_MSPC_FRAME_FPORT = 0x31, // MSP client frame
    FSSP_MSPS_FRAME = 0x32, // MSP server frame

    // ID of sensor. Must be something that is polled by FrSky RX
    FSSP_SENSOR_ID1 = 0x1B,
    FSSP_SENSOR_ID2 = 0x0D,
    FSSP_SENSOR_ID3 = 0x34,
    FSSP_SENSOR_ID4 = 0x67
    // there are 32 ID's polled by smartport master
    // remaining 3 bits are crc (according to comments in openTx code)
};

typedef struct smartPortPayload_s {
    uint8_t  frameId;
    uint16_t valueId;
    uint32_t data;
} __attribute__((packed)) smartPortPayload_t;

typedef void smartPortWriteFrameFn(const smartPortPayload_t *payload);
typedef bool smartPortCheckQueueEmptyFn(void);

//bool initSmartPortTelemetry(void);
//void checkSmartPortTelemetryState(void);
bool initSmartPortTelemetryExternal(smartPortWriteFrameFn *smartPortWriteFrameExternal);

//void handleSmartPortTelemetry(void);
void processSmartPortTelemetry(smartPortPayload_t *payload, volatile bool *hasRequest, const uint32_t *requestTimeout);

smartPortPayload_t *smartPortDataReceive(uint16_t c, bool *clearToSend, smartPortCheckQueueEmptyFn *checkQueueEmpty, bool withChecksum);

//struct serialPort_s;
//void smartPortWriteFrameSerial(const smartPortPayload_t *payload, struct serialPort_s *port, uint16_t checksum);
//void smartPortSendByte(uint8_t c, uint16_t *checksum, struct serialPort_s *port);
//bool smartPortPayloadContainsMSP(const smartPortPayload_t *payload);
