/*
 * Copyright (C) Bart Slinger
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
 * @file "modules/loggers/sdlogger_spi_direct.c"
 * @author Bart Slinger
 * SPI SD Logger that saves pprzlog messages to SD Card.
 *
 * Developed using Test Driven Development.
 * Test code available at:
 *   https://github.com/bartslinger/paparazzi-unittest
 */

#define PERIODIC_C_LOGGER

#include "modules/loggers/sdlogger_spi_direct.h"
#include "subsystems/datalink/downlink.h"
#include "modules/loggers/pprzlog_tp.h"
#include "subsystems/datalink/telemetry.h"
#include "led.h"

#include <stdbool.h>

#if SDLOGGER_ON_ARM
#include "autopilot.h"
#endif

#ifdef LOGGER_LED
#define LOGGER_LED_ON LED_ON(LOGGER_LED);
#define LOGGER_LED_OFF LED_OFF(LOGGER_LED);
#else
#define LOGGER_LED_ON {}
#define LOGGER_LED_OFF {}
#endif

#ifndef TELEMETRY_MODE_Main_empty
#warning You need to define a main telemetry mode named "empty" without any \
  messages in your config file in /conf/telemetry/<your_config.xml>. \
  \
  Add <mode name="empty"></mode> to your main telemetry process.
#endif

#ifndef TELEMETRY_PROCESS_Logger
#error "You need to use a telemetry xml file with Logger process!"
#endif

#ifndef SDLOGGER_DOWNLOAD_DEVICE
#error No downlink device defined for SD Logger
#endif

#ifdef SDLOGGER_DOWNLOAD_DEVICE_LISTEN
// Listen for setting commands on download port
#include "pprzlink/dl_protocol.h"
#include "generated/settings.h"

#include <string.h>

PRINT_CONFIG_MSG("Listening to SETTING on SD logger download port.");

static struct download_port_t {
  struct link_device *device;
  struct pprz_transport transport;
  bool msg_available;
  uint8_t msg_buf[256];
} download_port;

static void sdlogger_spi_direct_download_port_init(void) {
  download_port.device = &((SDLOGGER_DOWNLOAD_DEVICE).device);
  pprz_transport_init(&download_port.transport);
}

static void sdlogger_spi_direct_download_port_parse_msg(void) {
  switch (IdOfPprzMsg(download_port.msg_buf)) {
    case DL_SETTING: {
      uint8_t index = DL_SETTING_index(download_port.msg_buf);
      uint8_t ac_id = DL_SETTING_ac_id(download_port.msg_buf);
      float value = DL_SETTING_value(download_port.msg_buf);
      if (ac_id == AC_ID) {
        DlSetting(index, value);
        pprz_msg_send_DL_VALUE(&download_port.transport.trans_tx, download_port.device, AC_ID, &index, &value);
        sdlogger_spi_direct_command();
      }
      break; }
    default:
      break;
  }
}

static void sdlogger_spi_direct_download_port_periodic(void) {
  pprz_check_and_parse(download_port.device, &download_port.transport, download_port.msg_buf, &download_port.msg_available);
  if (download_port.msg_available) {
    sdlogger_spi_direct_download_port_parse_msg();
    download_port.msg_available = false;
  }
}
#endif // SDLOGGER_DOWNLOAD_DEVICE_LISTEN

struct sdlogger_spi_periph sdlogger_spi;

/**
 * @brief sdlogger_spi_direct_init
 * Initialize the logger and SD Card.
 */
void sdlogger_spi_direct_init(void)
{
  /* Initialize the SD Card */
  sdcard_spi_init(&sdcard1, &(SDLOGGER_SPI_LINK_DEVICE),
                  SDLOGGER_SPI_LINK_SLAVE_NUMBER);

  /* Set values in the struct to their defaults */
  sdlogger_spi.status = SDLogger_Initializing;
  sdlogger_spi.next_available_address = 0;
  sdlogger_spi.last_completed = 0;
  sdlogger_spi.sdcard_buf_idx = 1;

  /* Fill internal buffer with zeros */
  for (uint8_t i = 0; i < sizeof(sdlogger_spi.buffer); i++) {
    sdlogger_spi.buffer[i] = 0;
  }
  sdlogger_spi.idx = 0;
  sdlogger_spi.log_len = 0;
  sdlogger_spi.command = 0;
  sdlogger_spi.download_id = 0;
  sdlogger_spi.download_address = 0;
  sdlogger_spi.download_length = 0;
  sdlogger_spi.do_log = 0;

  /* Set function pointers in link_device to the logger functions */
  sdlogger_spi.device.check_free_space = (check_free_space_t)sdlogger_spi_direct_check_free_space;
  sdlogger_spi.device.put_byte = (put_byte_t)sdlogger_spi_direct_put_byte;
  sdlogger_spi.device.put_buffer = (put_buffer_t)sdlogger_spi_direct_put_buffer;
  sdlogger_spi.device.send_message = (send_message_t)sdlogger_spi_direct_send_message;
  sdlogger_spi.device.char_available = (char_available_t)sdlogger_spi_direct_char_available;
  sdlogger_spi.device.get_byte = (get_byte_t)sdlogger_spi_direct_get_byte;
  sdlogger_spi.device.periph = &sdlogger_spi;

#ifdef SDLOGGER_DOWNLOAD_DEVICE_LISTEN
  sdlogger_spi_direct_download_port_init();
#endif
}

/**
 * @brief sdlogger_spi_direct_periodic
 * Periodic function called at module frequency
 */
void sdlogger_spi_direct_periodic(void)
{
#ifdef SDLOGGER_DOWNLOAD_DEVICE_LISTEN
  sdlogger_spi_direct_download_port_periodic();
#endif

  sdcard_spi_periodic(&sdcard1);

#if SDLOGGER_ON_ARM
  if(autopilot_get_motors_on()) {
    sdlogger_spi.do_log = 1;
  } else {
    sdlogger_spi.do_log = 0;
  }
#endif

  switch (sdlogger_spi.status) {
    case SDLogger_Initializing:
      if (sdcard1.status == SDCard_Idle) {
        sdcard_spi_read_block(&sdcard1, 0x00002000, &sdlogger_spi_direct_index_received);
        sdlogger_spi.status = SDLogger_RetreivingIndex;
      }
      break;

    case SDLogger_Ready:
      if ((sdlogger_spi.do_log == 1) &&
          sdcard1.status == SDCard_Idle) {
        LOGGER_LED_ON;
        sdcard_spi_multiwrite_start(&sdcard1, sdlogger_spi.next_available_address);
        sdlogger_spi.status = SDLogger_Logging;
      }
      break;

    case SDLogger_Logging:
      /* This line is NOT unit-tested because it is an inline function */
      #if PERIODIC_TELEMETRY
      periodic_telemetry_send_Logger(DefaultPeriodic,
                                     &pprzlog_tp.trans_tx,
                                     &sdlogger_spi.device);
      #endif
      /* Check if SD Card buffer is full and SD Card is ready for new data */
      if (sdlogger_spi.sdcard_buf_idx > 512 &&
          sdcard1.status == SDCard_MultiWriteIdle) {
        sdcard_spi_multiwrite_next(&sdcard1, &sdlogger_spi_direct_multiwrite_written);
      }
      /* Check if switch is flipped to stop logging */
      if (sdlogger_spi.do_log == 0) {
        sdlogger_spi.status = SDLogger_LoggingFinalBlock;
      }
      break;

    case SDLogger_LoggingFinalBlock:
      if (sdcard1.status == SDCard_MultiWriteIdle) {
        if (sdlogger_spi.sdcard_buf_idx > 512) {
          sdcard_spi_multiwrite_next(&sdcard1, &sdlogger_spi_direct_multiwrite_written);
        }
        else if (sdlogger_spi.sdcard_buf_idx > 1) {
          /* Fill with trailing zero's */
          for (uint16_t i = sdlogger_spi.sdcard_buf_idx; i < (SD_BLOCK_SIZE+1); i++) {
            sdcard1.output_buf[i] = 0x00;
          }
          sdcard_spi_multiwrite_next(&sdcard1, &sdlogger_spi_direct_multiwrite_written);
        }
        else if (sdlogger_spi.sdcard_buf_idx == 1) {
          sdcard_spi_multiwrite_stop(&sdcard1);
          sdlogger_spi.status = SDLogger_StoppedLogging;
        }
      }
      break;

    case SDLogger_StoppedLogging:
      if (sdcard1.status == SDCard_Idle) {
        sdcard_spi_read_block(&sdcard1, 0x00002000, &sdlogger_spi_direct_index_received);
        sdlogger_spi.status = SDLogger_GettingIndexForUpdate;
      }
      break;

    case SDLogger_UpdatingIndex:
      if (sdcard1.status == SDCard_Idle) {
        LOGGER_LED_OFF;
        sdlogger_spi.status = SDLogger_Ready;
      }
      break;

    case SDLogger_Downloading:
      if (sdcard1.status == SDCard_Idle) {
        /* Put bytes to the buffer until all is written or buffer is full */
        long fd = 0;
        uint16_t chunk_size = 64;
        for (uint16_t i = sdlogger_spi.sdcard_buf_idx; i < SD_BLOCK_SIZE && chunk_size > 0; i++, chunk_size--) {
          if ((SDLOGGER_DOWNLOAD_DEVICE).device.check_free_space(&(SDLOGGER_DOWNLOAD_DEVICE), &fd, 1)) {
            (SDLOGGER_DOWNLOAD_DEVICE).device.put_byte(&(SDLOGGER_DOWNLOAD_DEVICE), fd, sdcard1.input_buf[i]);
          } else {
            /* No free space left, abort for-loop */
            break;
          }
          sdlogger_spi.sdcard_buf_idx++;
        }
        /* Request next block if entire buffer was written to uart */
        if (sdlogger_spi.sdcard_buf_idx >= SD_BLOCK_SIZE) {
          (SDLOGGER_DOWNLOAD_DEVICE).device.send_message(&(SDLOGGER_DOWNLOAD_DEVICE), fd);  // Flush buffers
          if (sdlogger_spi.download_length > 0) {
            sdcard_spi_read_block(&sdcard1, sdlogger_spi.download_address, NULL);
            sdlogger_spi.download_address++;
            sdlogger_spi.download_length--;
          }
          else {
            LOGGER_LED_OFF;
            sdlogger_spi.status = SDLogger_Ready;
          }
          sdlogger_spi.sdcard_buf_idx = 0;
        }
      }
      break;

    default:
      break;
  }
}

void sdlogger_spi_direct_start(void) {}
void sdlogger_spi_direct_stop(void) {}

/**
 * @brief sdlogger_spi_direct_index_received
 * Callback from SD Card when block at index location is received.
 */
void sdlogger_spi_direct_index_received(void)
{

  switch (sdlogger_spi.status) {
    case SDLogger_RetreivingIndex:
      sdlogger_spi.next_available_address = 0x00004000;
      sdlogger_spi.last_completed = 0;
      // Save data for later use
      sdlogger_spi.next_available_address = (sdcard1.input_buf[0] << 24) |
                                            (sdcard1.input_buf[1] << 16) |
                                            (sdcard1.input_buf[2] << 8) |
                                            (sdcard1.input_buf[3]);
      sdlogger_spi.last_completed = sdcard1.input_buf[4];

      if(sdlogger_spi.next_available_address < 0x00004000) {
        sdlogger_spi.next_available_address = 0x00004000;
      }

      /* Ready to start logging */
      sdlogger_spi.status = SDLogger_Ready;
      break;

    case SDLogger_GettingIndexForUpdate:
      /* Copy input buffer to output buffer */
      for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++) {
        sdcard1.output_buf[i+6] = sdcard1.input_buf[i];
      }

      /* Increment last completed log */
      sdcard1.output_buf[4+6] = ++sdlogger_spi.last_completed;
      /* Write log info at dedicated location */
      {
        uint16_t log_idx_start = 5 + 6 + (sdlogger_spi.last_completed - 1) * 12;

        /* Set start address and length at location that belongs to the log nr */
        sdcard1.output_buf[log_idx_start+0] = sdlogger_spi.next_available_address >> 24;
        sdcard1.output_buf[log_idx_start+1] = sdlogger_spi.next_available_address >> 16;
        sdcard1.output_buf[log_idx_start+2] = sdlogger_spi.next_available_address >> 8;
        sdcard1.output_buf[log_idx_start+3] = sdlogger_spi.next_available_address >> 0;
        sdcard1.output_buf[log_idx_start+4] = sdlogger_spi.log_len >> 24;
        sdcard1.output_buf[log_idx_start+5] = sdlogger_spi.log_len >> 16;
        sdcard1.output_buf[log_idx_start+6] = sdlogger_spi.log_len >> 8;
        sdcard1.output_buf[log_idx_start+7] = sdlogger_spi.log_len >> 0;
      }

      /* Increment and update the next available address */
      sdlogger_spi.next_available_address += sdlogger_spi.log_len;
      sdcard1.output_buf[0+6] = sdlogger_spi.next_available_address >> 24;
      sdcard1.output_buf[1+6] = sdlogger_spi.next_available_address >> 16;
      sdcard1.output_buf[2+6] = sdlogger_spi.next_available_address >> 8;
      sdcard1.output_buf[3+6] = sdlogger_spi.next_available_address >> 0;

      sdcard_spi_write_block(&sdcard1, 0x00002000);
      /* Reset log length */
      sdlogger_spi.log_len = 0;
      sdlogger_spi.status = SDLogger_UpdatingIndex;
      break;

    case SDLogger_GettingIndexForDownload:
      {
        uint16_t info_idx = 5 + (sdlogger_spi.download_id - 1) * 12;
        sdlogger_spi.download_address = (sdcard1.input_buf[info_idx+0] << 24) |
                                        (sdcard1.input_buf[info_idx+1] << 16) |
                                        (sdcard1.input_buf[info_idx+2] << 8) |
                                        (sdcard1.input_buf[info_idx+3] << 0);
        sdlogger_spi.download_length = (sdcard1.input_buf[info_idx+4] << 24) |
                                       (sdcard1.input_buf[info_idx+5] << 16) |
                                       (sdcard1.input_buf[info_idx+6] << 8) |
                                       (sdcard1.input_buf[info_idx+7] << 0);
        if (sdlogger_spi.download_length > 0) {
          /* Request the first block */
          sdcard_spi_read_block(&sdcard1, sdlogger_spi.download_address, NULL);
          /* After each read block, incr address, decr length */
          sdlogger_spi.download_address++;
          sdlogger_spi.download_length--;
          sdlogger_spi.status = SDLogger_Downloading;
        }
        else {
          LOGGER_LED_OFF;
          sdlogger_spi.status = SDLogger_Ready;
        }
        sdlogger_spi.sdcard_buf_idx = 0;
      }
      break;

    default:
      break;
  }

}

/**
 * @brief sdlogger_spi_direct_multiwrite_written
 * Called when a multiwrite is complete. Data stored in the logger buffer is
 * then moved to the SD Card buffer, which is now available again.
 */
void sdlogger_spi_direct_multiwrite_written(void)
{
  /* Increment log length */
  sdlogger_spi.log_len++;

  /* Copy data from logger buffer to SD Card buffer */
  for (uint8_t i = 0; i < sdlogger_spi.idx; i++) {
    sdcard1.output_buf[i+1] = sdlogger_spi.buffer[i];
  }
  /* Set sdcard buffer index to new value */
  sdlogger_spi.sdcard_buf_idx = sdlogger_spi.idx + 1;
  /* And reset the logger buffer index */
  sdlogger_spi.idx = 0;
}

void sdlogger_spi_direct_command(void)
{
  if (sdcard1.status == SDCard_Idle && sdlogger_spi.command > 0 &&
      sdlogger_spi.command < 43) {
    LOGGER_LED_ON;
    sdcard_spi_read_block(&sdcard1, 0x00002000,
                          &sdlogger_spi_direct_index_received);
    sdlogger_spi.download_id = sdlogger_spi.command;
    sdlogger_spi.status = SDLogger_GettingIndexForDownload;
  }
  else if (sdcard1.status == SDCard_Idle && sdlogger_spi.command == 255) {
    telemetry_mode_Main = TELEMETRY_MODE_Main_empty;
    LOGGER_LED_ON;
    sdcard_spi_read_block(&sdcard1, 0x00002000, NULL);
    sdlogger_spi.download_length = 0;
    sdlogger_spi.sdcard_buf_idx = 0;
    sdlogger_spi.status = SDLogger_Downloading;
  }
  /* Always reset command value back to zero */
  sdlogger_spi.command = 0;
}

int sdlogger_spi_direct_check_free_space(struct sdlogger_spi_periph *p, long *fd __attribute__((unused)), uint16_t len)
{
  if (p->status == SDLogger_Logging) {
    /* Calculating free space in both buffers */
    int available = (513 - p->sdcard_buf_idx) + (SDLOGGER_BUFFER_SIZE - p->idx);
    if (available >= len) {
      return available;
    }
  }
  return 0;
}

void sdlogger_spi_direct_put_byte(struct sdlogger_spi_periph *p, long fd __attribute__((unused)), uint8_t data)
{
  /* SD Buffer full, write in logger buffer */
  if (p->sdcard_buf_idx > 512) {
    if (p->idx < SDLOGGER_BUFFER_SIZE) {
      p->buffer[p->idx++] = data;
    }
    /* else: data lost */
  }
  /* Writing directly to SD Card buffer */
  else {
    sdcard1.output_buf[p->sdcard_buf_idx++] = data;

    /* Flush buffer */
    if (p->sdcard_buf_idx > 512 && sdcard1.status == SDCard_MultiWriteIdle) {
      sdcard_spi_multiwrite_next(&sdcard1, &sdlogger_spi_direct_multiwrite_written);
    }
  }
}

void sdlogger_spi_direct_put_buffer(struct sdlogger_spi_periph *p, long fd, uint8_t *data, uint16_t len)
{
  int i;
  for (i = 0; i < len; i++) {
    sdlogger_spi_direct_put_byte(p, fd, data[i]);
  }
}

void sdlogger_spi_direct_send_message(void *p, long fd __attribute__((unused)))
{
  (void) p;
}

int sdlogger_spi_direct_char_available(void *p){
  (void) p;
  return 0;
}

uint8_t sdlogger_spi_direct_get_byte(void *p)
{
  (void) p;
  return 0;
}

