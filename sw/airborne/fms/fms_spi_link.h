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
};

struct SpiLink spi_link;

extern int spi_link_init(void);
extern int spi_link_send(const void *buf_out, size_t count, void* buf_in);

#endif /* FMS_SPI_LINK_H */
