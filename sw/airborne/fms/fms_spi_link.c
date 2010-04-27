#include "fms_spi_link.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

int spi_link_init(void) {

  spi_link.device = "/dev/spidev1.1";
  //  spi_link.mode  = 0;
  //  spi_link.mode  = SPI_CPHA | SPI_CPOL | SPI_LSB_FIRST;
  spi_link.mode  = SPI_CPHA;
  //  spi_link.mode  = SPI_LSB_FIRST;
  spi_link.bits  = 8;
  spi_link.speed = 500000;
  spi_link.delay = 1;

  spi_link.fd = open(spi_link.device, O_RDWR);
  if (spi_link.fd < 0)
    return -1;

  int ret = 0;
  ret = ioctl(spi_link.fd, SPI_IOC_WR_MODE, &spi_link.mode);
  if (ret == -1)
    return -2;

  ret = ioctl(spi_link.fd, SPI_IOC_WR_BITS_PER_WORD, &spi_link.bits);
  if (ret == -1)
    return -3;

  ret = ioctl(spi_link.fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_link.speed);
  if (ret == -1)
    return -4;

  return 0;
}

int spi_link_send(const void *buf_out, size_t count, void *buf_in) {

 int ret; 

 struct spi_ioc_transfer tr = {
   .tx_buf = (unsigned long)buf_out,
   .rx_buf = (unsigned long)buf_in,
   .len = count,
   .delay_usecs = spi_link.delay,
   .speed_hz = spi_link.speed,
   .bits_per_word = spi_link.bits,
 };
 ret = ioctl(spi_link.fd, SPI_IOC_MESSAGE(1), &tr);

 return ret;

}
