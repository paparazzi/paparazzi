
#include "fms/fms_crc.h"

crc_t crc__table[256];

void crc__init(uint32_t polynomial) {
  crc_t crc_remainder;
  uint32_t crc_dividend;
  crc_t top_bit = (1 << (CRC__WIDTH - 1));
  uint8_t bit;
  for(crc_dividend = 0; crc_dividend < 256; crc_dividend++) {
    crc_remainder = crc_dividend << (CRC__WIDTH - 8);
    for(bit = 8; bit > 0; bit--) {
      if(crc_remainder & top_bit) {
	crc_remainder = (crc_remainder << 1) ^ polynomial;
      }
      else {
	crc_remainder = (crc_remainder << 1);
      }
    }
    crc__table[crc_dividend] = crc_remainder;
  }

#if 0
  int i=0;
  while (i<256) {
    printf("%03d ",crc__table[i]);
    if ((i%8)==7) printf("\n");
    i++;
  }
#endif

}

#define POLYNOMIAL 0x31
#define WIDTH  (8 * sizeof(uint8_t))
#define TOPBIT (1 << (WIDTH - 1))
uint8_t crc__calc_block_crc8(const uint8_t buf[], uint32_t len) {
  crc_t  _remainder = 0;	
  for (int byte = 0; byte < len; ++byte)  {
    _remainder ^= (buf[byte] << (WIDTH - 8));
    for (uint8_t bit = 8; bit > 0; --bit)  {
      if (_remainder & TOPBIT)
	_remainder = (_remainder << 1) ^ POLYNOMIAL;
      else
	_remainder = (_remainder << 1);
    }
  }
  return (_remainder);
} 

#if 0
uint8_t crc__calc_block_crc8(const uint8_t buffer[], uint32_t buffer_length) {
  int counter;
  uint16_t crc = 0;
  for(counter = 0; counter < buffer_length; counter++) {
    crc = crc ^ crc__table[ ( crc ^ *(char *)(buffer)++ ) & 0x00FF ];
  }
  return crc;
}
#endif


uint16_t crc__calc_block_crc16(const void * buffer, uint32_t buffer_length) {
  int counter;
  crc_t crc = 0;
  for(counter = 0; counter < buffer_length; counter++) {
    crc = (crc << 8) ^ crc__table[ ( (crc >> 8) ^ *(char *)(buffer)++ ) & 0x00FF ];
  }
  return crc;
}



