/*
 * Serial_Communication.h
 *
 *  Created on: Jul 25, 2017
 *      Author: Steven van der Helm
 */

#ifndef DECAWAVE_SERIAL_COMMUNICATION_H_
#define DECAWAVE_SERIAL_COMMUNICATION_H_

#include <stdint.h>
#include <stdbool.h>

extern void decawave_serial_communication_init(void);
extern void decawave_serial_communication_periodic(void);
extern void decawave_serial_communication_event(void);

extern void getSerialData(void);
extern void sendFloat(uint8_t msgtype, float outfloat);

#endif /* DECAWAVE_SERIAL_COMMUNICATION_H_ */
