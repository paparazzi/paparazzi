/*
 * Serial_Communication.h
 *
 *  Created on: Jul 25, 2017
 *      Author: Steven van der Helm
 */

#ifndef DECAWAVE_SERIAL_SERIAL_COMMUNICATION_H_
#define DECAWAVE_SERIAL_SERIAL_COMMUNICATION_H_

#include <stdint.h>
#include <stdbool.h>

// Some meta data for serial communication
#define UWB_SERIAL_COMM_MAX_MESSAGE 10
#define IN_MESSAGE_SIZE 4
#define OUT_MESSAGE_SIZE 7
#define UWB_SERIAL_COMM_END_MARKER 255
#define UWB_SERIAL_COMM_SPECIAL_BYTE 253
#define UWB_SERIAL_COMM_START_MARKER 254
#define UWB_SERIAL_COMM_NODE_STATE_SIZE 4

// Size of a floating point number
#define UWB_SERIAL_COMM_FLOAT_SIZE 4

// Setting up how many nodes can maximally be in the network
#define UWB_SERIAL_COMM_MAX_NODES 5

// How many nodes actually are in the network
#define UWB_SERIAL_COMM_NUM_NODES 3

// How many distant nodes are in the network (one less than the toal number of nodes)
#define UWB_SERIAL_COMM_DIST_NUM_NODES UWB_SERIAL_COMM_NUM_NODES-1

// Serial message types
#define UWB_SERIAL_COMM_VX 0
#define UWB_SERIAL_COMM_VY 1
#define UWB_SERIAL_COMM_Z 2
#define UWB_SERIAL_COMM_RANGE 3

extern void decawave_serial_communication_init(void);
extern void decawave_serial_communication_periodic(void);
extern void decawave_serial_communication_event(void);

extern void getSerialData(void);
extern void sendFloat(uint8_t msgtype, float outfloat);

#endif /* DECAWAVE_SERIAL_SERIAL_COMMUNICATION_H_ */
