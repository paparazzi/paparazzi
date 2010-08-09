#ifndef __CRC_H__
#define __CRC_H__

#include "std.h"

typedef uint8_t crc_t;

#define CRC__WIDTH (8 * sizeof(crc_t))

void crc__init(uint32_t polynomial);

extern uint8_t crc__calc_block_crc8(const uint8_t buffer[], uint32_t buffer_length);
uint16_t crc__calc_block_crc16(const void * buffer,
                               uint32_t buffer_length);

#define crc8__check(buffer, length, crc) \
	(crc == crc__calc_block_crc8((buffer), (length)));
#define crc16__check(buffer, length, crc) \
	(crc == crc__calc_block_crc16((buffer), (length)));

#endif /* __CRC_H__ */
