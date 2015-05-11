/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file arch/linux/mcu_periph/adc_arch.c
 * @ingroup linux_arch
 *
 * Driver for the analog to digital converters in Linux based systems.
 */

#include "mcu_periph/adc.h"

#include BOARD_CONFIG
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>


/******************************/
/***   INTERNAL VARIABLES   ***/
/******************************/

/* ADC0 */
#if USE_ADC0
static uint8_t adc0_channels[] = {ADC0_CHANNELS};
struct adc_t adc0 = {
  .dev_id = ADC0_ID,
  .channels = adc0_channels,
  .channels_cnt = ADC0_CHANNELS_CNT,
  .buf_length = ADC0_BUF_LENGTH
};
#endif

/* ADC1 */
#if USE_ADC1
static uint8_t adc1_channels[] = {ADC1_CHANNELS};
struct adc_t adc1 = {
  .dev_id = ADC1_ID,
  .channels = adc1_channels,
  .channels_cnt = ADC1_CHANNELS_CNT,
  .buf_length = ADC1_BUF_LENGTH
};
#endif

/* Private functions */
static inline void adc_dev_init(struct adc_t *adc);
static void write_sysfs_int(uint8_t dev_id, char *filename, int val);

/***************************************/
/***   PUBLIC FUNCTION DEFINITIONS   ***/
/***************************************/

/**
 * Initialize the ADC
 */
void adc_init(void)
{
#if USE_ADC0
  adc_dev_init(&adc0);
#endif
#if USE_ADC1
  adc_dev_init(&adc1);
#endif
}

/**
 * @todo: fx a more general ADC
 */
void adc_buf_channel(uint8_t adc_channel, struct adc_buf *s, uint8_t av_nb_sample)
{

}

/**
 * Start or stop the ADC readings
 * @param[in] *adc The ADC to start the readings for
 * @param[in] value 1 to enable and 0 to disable
 */
void adc_enable(struct adc_t *adc, uint8_t value)
{
  /* Write 1 or 0 to enable/disable the ADC */
  write_sysfs_int(adc->dev_id, "buffer/enable", value);
}

/**
 * Read the ADC buffer from the driver
 * @param[in] *adc The adc you want to read from
 * @param[out] *buf Output values
 * @param[in] size The amount of bytes you want to read
 */
int adc_read(struct adc_t *adc, uint16_t *buf, uint16_t size)
{
  /* Allocate dev_id + name */
  char *temp;
  if(asprintf(&temp, "/dev/iio:device%d", adc->dev_id) < 0) {
    return -1;
  }

  /* Open the file */
  int fd = open(temp, O_RDONLY | O_NONBLOCK);
  free(temp);

  if(fd < 0) {
    return -2;
  }

  struct pollfd pfd;
  pfd.fd = fd;
  pfd.events = POLLIN;
  poll(&pfd, 1, -1);

  /* Read the file */
  int ret = read(fd, buf, size);
  close(fd);
  return ret;
}

/****************************************/
/***   PRIVATE FUNCTION DEFINITIONS   ***/
/****************************************/

/**
 * Initialize an ADC device
 * @param[in] *adc The ADC device to initialize
 */
static inline void adc_dev_init(struct adc_t *adc)
{
  char filename[32];
  uint8_t i;

  /* Enable all linked channels */
  for(i = 0; i < adc->channels_cnt; i++) {
    sprintf(filename, "scan_elemens/in_voltage%d_en", adc->channels[i]);
    write_sysfs_int(adc->dev_id, filename, 1);
  }

  /* Set the buffer length */
  write_sysfs_int(adc->dev_id, "buffer/length", adc->buf_length);
}

/**
 * Write an int to a sysfs file
 * @param[in] dev_id The device id
 * @param[in] *filename The file to write to
 * @param[in] val The value to write
 */
static void write_sysfs_int(uint8_t dev_id, char *filename, int val)
{
  /* Allocate dev_id + filename */
  char *temp;
  if (asprintf(&temp, "/sys/bus/iio/devices/iio:device%d/%s", dev_id, filename) < 0) {
    return;
  }

  /* Open the file */
  FILE *fd = fopen(temp, "w");
  free(temp);

  if (fd == NULL) {
    return;
  }

  /* Write the value to the file */
  fprintf(fd, "%d", val);
  fclose(fd);
}
