#include "fms_spi_link.h"

#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>


int spi_link_init(void)
{

  spi_link.device = "/dev/spidev1.1";
  spi_link.mode  = SPI_CPHA;
  spi_link.bits  = 8;
  spi_link.speed = 12000000;
  spi_link.delay = 1;

  spi_link.msg_cnt = 0;
  spi_link.crc_err_cnt = 0;

  spi_link.fd = open(spi_link.device, O_RDWR);
  if (spi_link.fd < 0) {
    return -1;
  }

  int ret = 0;
  ret = ioctl(spi_link.fd, SPI_IOC_WR_MODE, &spi_link.mode);
  if (ret == -1) {
    return -2;
  }

  ret = ioctl(spi_link.fd, SPI_IOC_WR_BITS_PER_WORD, &spi_link.bits);
  if (ret == -1) {
    return -3;
  }

  ret = ioctl(spi_link.fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_link.speed);
  if (ret == -1) {
    return -4;
  }

  return 0;
}

int spi_link_send(void *buf_out, size_t count, void *buf_in, uint8_t *crc_valid)
{

  int ret;

  struct spi_ioc_transfer tr = {
    .tx_buf        = (unsigned long)buf_out,
    .rx_buf        = (unsigned long)buf_in,
    .len           = count,
    .delay_usecs   = spi_link.delay,
    .speed_hz      = spi_link.speed,
    .bits_per_word = spi_link.bits,
  };

  ((uint8_t *)buf_out)[count - 1] = crc_calc_block_crc8(buf_out, count - 1);
  ret = ioctl(spi_link.fd, SPI_IOC_MESSAGE(1), &tr);
  spi_link.msg_cnt++;

  uint8_t computed_crc = crc_calc_block_crc8(buf_in, count - 1);
  if (computed_crc == ((uint8_t *)buf_in)[count - 1]) {
    *crc_valid = 1;
  } else {
    *crc_valid = 0;
    spi_link.crc_err_cnt++;
  }

  return ret;

}


#define POLYNOMIAL 0x31
#define WIDTH  (8 * sizeof(uint8_t))
#define TOPBIT (1 << (WIDTH - 1))
uint8_t crc_calc_block_crc8(const uint8_t buf[], uint32_t len)
{
  uint8_t  _remainder = 0;
  for (uint32_t byte = 0; byte < len; ++byte)  {
    _remainder ^= (buf[byte] << (WIDTH - 8));
    for (uint8_t bit = 8; bit > 0; --bit)  {
      if (_remainder & TOPBIT) {
        _remainder = (_remainder << 1) ^ POLYNOMIAL;
      } else {
        _remainder = (_remainder << 1);
      }
    }
  }
  return (_remainder);
}


#if 0
/* for reference: need to write a more efficient crc computation */
crc_t crc__table[256];

void crc__init(uint32_t polynomial)
{
  crc_t crc_remainder;
  uint32_t crc_dividend;
  crc_t top_bit = (1 << (CRC__WIDTH - 1));
  uint8_t bit;
  for (crc_dividend = 0; crc_dividend < 256; crc_dividend++) {
    crc_remainder = crc_dividend << (CRC__WIDTH - 8);
    for (bit = 8; bit > 0; bit--) {
      if (crc_remainder & top_bit) {
        crc_remainder = (crc_remainder << 1) ^ polynomial;
      } else {
        crc_remainder = (crc_remainder << 1);
      }
    }
    crc__table[crc_dividend] = crc_remainder;
  }

#if 0
  int i = 0;
  while (i < 256) {
    printf("%03d ", crc__table[i]);
    if ((i % 8) == 7) { printf("\n"); }
    i++;
  }
#endif

}

uint8_t crc__calc_block_crc8(const uint8_t buffer[], uint32_t buffer_length)
{
  int counter;
  uint16_t crc = 0;
  for (counter = 0; counter < buffer_length; counter++) {
    crc = crc ^ crc__table[(crc ^ * (char *)(buffer)++) & 0x00FF ];
  }
  return crc;
}


#endif
