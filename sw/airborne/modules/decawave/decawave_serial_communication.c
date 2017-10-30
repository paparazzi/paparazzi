/*
 * Serial_Communication.c
 *
 *  Created on: Jul 25, 2017
 *      Author: Steven van der Helm
 */

/*
 * Copyright (C) C. DW
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/decawave_serial_communication.h"
 * @author S. vd H, C. DW
 *
 */

#include "decawave_serial_communication.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/radio_control.h"
#include "state.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/uart.h"
#include <stdio.h>
#include "subsystems/abi.h"

PRINT_CONFIG_VAR(SERIAL_UART)
PRINT_CONFIG_VAR(SERIAL_BAUD)

// define coms link for stereocam
#define SERIAL_PORT   (&((SERIAL_UART).device))
struct link_device *xdev = SERIAL_PORT;

#define SerialGetch() SERIAL_PORT ->get_byte(SERIAL_PORT->periph)
#define SerialSend1(c) SERIAL_PORT->put_byte(SERIAL_PORT->periph, 0, c)
#define SerialUartSend1(c) SerialSend1(c)
#define SerialSend(_dat,_len) { for (uint8_t i = 0; i< (_len); i++) SerialSend1(_dat[i]); };
#define SerialUartSetBaudrate(_b) uart_periph_set_baudrate(SERIAL_PORT, _b);
#define SerialChAvailable()(xdev->char_available(xdev->periph))
#define SerialSendNow() uart_send_message(SERIAL_PORT->periph,0)

struct nodeState{
	uint8_t nodeAddress;
	float vx;
	float vy;
	float z;
	float r;
	bool stateUpdated[UWB_SERIAL_COMM_NODE_STATE_SIZE];
};

static uint8_t _bytesRecvd = 0;
static uint8_t _dataRecvCount = 0;

static uint8_t _tempBuffer[UWB_SERIAL_COMM_MAX_MESSAGE];
static uint8_t _tempBuffer2[UWB_SERIAL_COMM_MAX_MESSAGE];
static uint8_t _recvBuffer[UWB_SERIAL_COMM_FLOAT_SIZE];

static uint8_t _dataSendCount = 0;
static uint8_t _dataTotalSend = 0;

static bool _inProgress = false;
static bool _allReceived = false;

static uint8_t _varByte = 0;

static struct nodeState _states[UWB_SERIAL_COMM_DIST_NUM_NODES];
float range_float = 0.0;

static void decodeHighBytes(void);
static void encodeHighBytes(uint8_t* sendData, uint8_t msgSize);
static void handleNewStateValue(uint8_t nodeIndex, uint8_t msgType, float value);
static void setNodeStatesFalse(uint8_t index);
static void setAllNodeStatesFalse(void);
static void checkStatesUpdated(void);

/**
 * Initialization function. Initializes nodes (which contain data of other bebops) and registers a periodic message.
 */
void decawave_serial_communication_init(void)
{
	setAllNodeStatesFalse();
}

/**
 * This function periodically sends state data over the serial (which is received by the arduino)
 */
void decawave_serial_communication_periodic(void)
{
	sendFloat(UWB_SERIAL_COMM_VX,stateGetSpeedEnu_f()->y);
	sendFloat(UWB_SERIAL_COMM_VY,stateGetSpeedEnu_f()->x);
	sendFloat(UWB_SERIAL_COMM_Z, stateGetPositionEnu_f()->z);
}

/**
 * Event function currently checks for serial data and whether an update of states is available for a distant drone.
 * If these cases are true, then actions are taken.
 */
void decawave_serial_communication_event(void){
	getSerialData();
	checkStatesUpdated();
}

/**
 * Helper function that sets the boolean that tells whether a remote drone has a new state update to false.
 */
static void setNodeStatesFalse(uint8_t index){
	for (uint8_t j = 0; j < UWB_SERIAL_COMM_NODE_STATE_SIZE; j++){
		_states[index].stateUpdated[j] = false;
	}
}

/**
 * Helper function that sets the booleans to false for all the remote drones (UWB_SERIAL_COMM_DIST_NUM_NODES)
 */
static void setAllNodeStatesFalse(void){
	for (uint8_t i = 0; i < UWB_SERIAL_COMM_DIST_NUM_NODES; i++){
		setNodeStatesFalse(i);
	}
}

/**
 * This function checks if all the states of all the distant nodes have at least once been updated.
 * If all the states are updated, then do something with it! AKA CALLBACK TO MARIO
 */
static void checkStatesUpdated(void){
	bool checkbool;
	for (uint8_t i = 0; i < UWB_SERIAL_COMM_DIST_NUM_NODES; i++){
		checkbool = true;
		for (uint8_t j = 0; j < UWB_SERIAL_COMM_NODE_STATE_SIZE; j++){
			checkbool = checkbool && _states[i].stateUpdated[j];
		}
		if (checkbool){
			AbiSendMsgUWB_COMMUNICATION(UWB_COMM_ID, i, _states[i].r, _states[i].vx, _states[i].vy, _states[i].z);
			printf("States for drone %i: r = %f, vx = %f, vy = %f, z = %f \n",i,_states[i].r,_states[i].vx,_states[i].vy,_states[i].z);
			setNodeStatesFalse(i);
		}
	}

}

/**
 * Function for receiving serial data.
 * Only receives serial data that is between the start and end markers. Discards all other data.
 * Stores the received data in _tempBuffer, and after decodes the high bytes and copies the final
 * message to the corresponding message in _messages.
 */
void getSerialData(void){
	while (SerialChAvailable()){
		_varByte = SerialGetch();
		if (_varByte == UWB_SERIAL_COMM_START_MARKER){
			_bytesRecvd = 0;
			_inProgress = true;
		}

		if (_inProgress){
			_tempBuffer[_bytesRecvd] = _varByte;
			_bytesRecvd++;
		}

		if (_varByte == UWB_SERIAL_COMM_END_MARKER){
			_inProgress = false;
			_allReceived = true;

			decodeHighBytes();
		}
	}
}

/**
 * Function for decoding the high bytes of received serial data and saving the message.
 * Since the start and end marker could also be regular payload bytes (since they are simply the values
 * 254 and 255, which could also be payload data) the payload values 254 and 255 have been encoded
 * as byte pairs 253 1 and 253 2 respectively. Value 253 itself is encoded as 253 0.
 *  This function will decode these back into values the original payload values.
 */
static void decodeHighBytes(void){
	_dataRecvCount = 0;
	float tempfloat;
	uint8_t thisAddress = _tempBuffer[1];
	uint8_t msgFrom = _tempBuffer[2];
	uint8_t msgType = _tempBuffer[3];
	uint8_t nodeIndex = msgFrom -1 - (uint8_t)(thisAddress<msgFrom);
	for (uint8_t i = 4; i<_bytesRecvd-1; i++){ // Skip the begin marker (0), this address (1), remote address (2), message type (3), and end marker (_bytesRecvd-1)
		_varByte = _tempBuffer[i];
		if (_varByte == UWB_SERIAL_COMM_SPECIAL_BYTE){
			i++;
			_varByte = _varByte + _tempBuffer[i];
		}
		if(_dataRecvCount<=UWB_SERIAL_COMM_FLOAT_SIZE){
			_recvBuffer[_dataRecvCount] = _varByte;
		}
		_dataRecvCount++;
	}
	if(_dataRecvCount==UWB_SERIAL_COMM_FLOAT_SIZE){
		memcpy(&tempfloat,&_recvBuffer,UWB_SERIAL_COMM_FLOAT_SIZE);
		handleNewStateValue(nodeIndex,msgType,tempfloat);
	}
}

/**
 * Function that is called when over the serial a new state value from a remote node is received
 */
static void handleNewStateValue(uint8_t nodeIndex, uint8_t msgType, float value){
	struct nodeState *node = &_states[nodeIndex];
	switch(msgType){
	case UWB_SERIAL_COMM_VX : node->vx=value; node->stateUpdated[UWB_SERIAL_COMM_VX] = true; break;
	case UWB_SERIAL_COMM_VY : node->vy=value; node->stateUpdated[UWB_SERIAL_COMM_VY] = true; break;
	case UWB_SERIAL_COMM_Z  : node->z=value ; node->stateUpdated[UWB_SERIAL_COMM_Z]  = true; break;
	case UWB_SERIAL_COMM_RANGE : node->r=value ; node->stateUpdated[UWB_SERIAL_COMM_RANGE]  = true; break;
	}
}

/**
 * Function that will send a float over serial. The actual message that will be sent will have
 * a start marker, the message type, 4 bytes for the float, and the end marker.
 */
void sendFloat(uint8_t msgtype, float outfloat){
	uint8_t floatbyte[4];
	memcpy(floatbyte,&outfloat,4);
	encodeHighBytes(floatbyte,4);
	SerialSend1(UWB_SERIAL_COMM_START_MARKER);
	SerialSend1(msgtype);
	SerialSend( _tempBuffer2,_dataTotalSend);
	SerialSend1(UWB_SERIAL_COMM_END_MARKER);
}

/**
 * Function that encodes the high bytes of the serial data to be sent.
 * Start and end markers are reserved values 254 and 255. In order to be able to send these values,
 * the payload values 253, 254, and 255 are encoded as 2 bytes, respectively 253 0, 253 1, and 253 2.
 */
static void encodeHighBytes(uint8_t* sendData, uint8_t msgSize){
	_dataSendCount = msgSize;
	_dataTotalSend = 0;
	for (uint8_t i = 0; i < _dataSendCount; i++){
		if (sendData[i] >= UWB_SERIAL_COMM_SPECIAL_BYTE){
			_tempBuffer2[_dataTotalSend] = UWB_SERIAL_COMM_SPECIAL_BYTE;
			_dataTotalSend++;
			_tempBuffer2[_dataTotalSend] = sendData[i] - UWB_SERIAL_COMM_SPECIAL_BYTE;
		}
		else{
			_tempBuffer2[_dataTotalSend] = sendData[i];
		}
		_dataTotalSend++;
	}
}
