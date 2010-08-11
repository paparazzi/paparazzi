#ifndef FMS_SPI_LINK_H
#define FMS_SPI_LINK_H

#include <inttypes.h>
#include <unistd.h>

struct SpiLink {
  int      fd;
  char*    device;
  uint8_t  mode;
  uint8_t  bits;
  uint32_t speed;
  uint16_t delay;
  /* number of message exchanged since initialization */
  uint32_t msg_cnt;
  /* number of crc errors on received messages        */
  uint32_t crc_err_cnt;
};

struct SpiLink spi_link;

/* 
 * initialize peripheral 
 */
extern int spi_link_init(void);

/* 
 *  exchange a data buffer
 *  the last byte of buf_out will be overrwiten with a crc
 *  the last byte of buf_in  will contain the received crc
 *  count is the size of buf_out and buf_in, that is
 *  the count of data to exchange+1
 */
extern int spi_link_send(void *buf_out, size_t count, void* buf_in, uint8_t* crc_valid);

/*
 * just for debuging purposes
 */
extern uint8_t crc_calc_block_crc8(const uint8_t buf[], uint32_t len);


#endif /* FMS_SPI_LINK_H */
