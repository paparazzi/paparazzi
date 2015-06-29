/*
 * Copyright (C) 2015 Bart Slinger <bartslinger@gmail.com>
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
 *
 */

/** @file modules/loggers/sd_logger.c
 *  @brief Module for real time logging using an SD Card in SPI mode.
 * This module buffers raw data from IMU and control inputs into a buffer of the SDCard peripheral.
 * Once the buffer is full, the entire data block is written to the card. No filesystem like FAT is used.
 * The card can be soldered directly to the SPI bus, so the increase in mass is minimal.
 * The logging is started and stopped by an external application in the ground station.
 * After logging, the data can be downloaded over telemetry messages using the same application in the ground station.
 *
 * The FIXME's in this file are discovered after a successful test of the module. They are not critical,
 * but will be fixed and tested in a next release.
 */

#include "peripherals/sdcard_spi.h"
#include "subsystems/datalink/telemetry.h"

#ifdef TEST
/* The original messages.h uses inline functions, which are incompatible with cmock. TEST is defined by the unittest framework */
#include "messages_testable.h"
#endif // TEST

#include "subsystems/imu.h"
#include "subsystems/actuators/actuators_pwm_arch.h"
#include "sd_logger_spi_direct.h"

/* Definition of the sdlogger */
struct SdLogger sdlogger;

/**
 * @brief Start function called during initialization of the autopilot.
 * It starts the initialization procedure of the SDCard.
 */
void sd_logger_start(void)
{
  sdcard_spi_init(&sdcard1, &(SD_LOGGER_SPI_LINK_DEVICE), SD_LOGGER_SPI_LINK_SLAVE_NUMBER);
  sdlogger.status = SdLogger_Initializing;
}

/**
 * @brief Periodic function of the logger, called at modules main frequency.
 */
void sd_logger_periodic(void)
{
  /* Keep the SDCard running at the same rate */
  sdcard_spi_periodic(&sdcard1);

  switch (sdlogger.status) {

      /* SD card is initializing, check to see if it is ready */
    case SdLogger_Initializing:
      if (sdcard1.status == SDCard_Error) {
        sdlogger.status = SdLogger_Error;
      } else if (sdcard1.status == SDCard_Idle) {
        sdlogger.status = SdLogger_Idle;
      }
      /* else: state unchanged */
      break;

      /* Logging data, write data to buffer */
    case SdLogger_Logging:
      sdlogger.packet_count++;
      sd_logger_uint32_to_buffer(sdlogger.packet_count, &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET + sdlogger.buffer_addr]);
      sd_logger_int32_to_buffer(imu.accel_unscaled.x,
                                &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET + sdlogger.buffer_addr + 4]);
      sd_logger_int32_to_buffer(imu.accel_unscaled.y,
                                &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET + sdlogger.buffer_addr + 8]);
      sd_logger_int32_to_buffer(imu.accel_unscaled.z,
                                &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET + sdlogger.buffer_addr + 12]);
      sd_logger_int32_to_buffer(imu.gyro_unscaled.p,
                                &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET + sdlogger.buffer_addr + 16]);
      sd_logger_int32_to_buffer(imu.gyro_unscaled.q,
                                &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET + sdlogger.buffer_addr + 20]);
      sd_logger_int32_to_buffer(imu.gyro_unscaled.r,
                                &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET + sdlogger.buffer_addr + 24]);
      sd_logger_int32_to_buffer(actuators_pwm_values[0],
                                &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET + sdlogger.buffer_addr + 28]);
      sd_logger_int32_to_buffer(actuators_pwm_values[2],
                                &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET + sdlogger.buffer_addr + 32]);
      sd_logger_int32_to_buffer(actuators_pwm_values[3],
                                &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET + sdlogger.buffer_addr + 36]);
      sd_logger_int32_to_buffer(actuators_pwm_values[4],
                                &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET + sdlogger.buffer_addr + 40]);
      sd_logger_int32_to_buffer(actuators_pwm_values[5],
                                &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET + sdlogger.buffer_addr + 44]);
      sd_logger_int32_to_buffer(0,
                                &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET + sdlogger.buffer_addr + 48]); // reserved for something
      sdlogger.buffer_addr += SD_LOGGER_PACKET_SIZE;

      /* Check if the buffer is now full. If so, write to SD card */
      if ((SD_BLOCK_SIZE - sdlogger.buffer_addr) < SD_LOGGER_PACKET_SIZE) {
        /* If the card is not idle, we cannot write the buffered data and it will be lost */
        if (sdcard1.status != SDCard_MultiWriteIdle) {
          sdlogger.error_count++;
        }
        /* Write the unique ID at the beginning of the block, to identify later if the block belongs to the dataset */
        sd_logger_uint32_to_buffer(sdlogger.unique_id, &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET]);
        /* Call the actual write function */
        sdcard_spi_multiwrite_next(&sdcard1);
        /* Reset the buffer address. First 4 bytes are reserved for the 32bit unique ID. */
        sdlogger.buffer_addr = SD_LOGGER_BLOCK_PREAMBLE_SIZE;
        /* Is unneccesary since multiwrite, but is kept because removing it is untested yet. */
        sdlogger.block_addr++;
      }
      break;

      /* Write summary block if sdcard is ready with writing the last data block. Always at address 0x00000000. */
    case SdLogger_WriteStatusPacket:
      if (sdcard1.status != SDCard_Idle) {
        break;
      }
      /* Values are written to buffer address 6+, since single write needs 6 bytes for the write command */
      /* Write number of packets */
      sd_logger_uint32_to_buffer(sdlogger.packet_count, &sdcard1.output_buf[6]);
      /* Write number of errors */
      sd_logger_uint32_to_buffer(sdlogger.error_count, &sdcard1.output_buf[6 + 4]);
      /* Write unique id of the measurement */
      sd_logger_uint32_to_buffer(sdlogger.unique_id, &sdcard1.output_buf[6 + 8]);
      /* Fill the rest of the block with zeros */
      for (uint16_t i = 12; i < SD_BLOCK_SIZE; i++) {
        sdcard1.output_buf[6 + i] = 0x00;
      }
      /* Write the status block */
      sdcard_spi_write_block(&sdcard1, 0x00000000);
      sdlogger.status = SdLogger_Idle;
      break;

      /* If SDCard is ready, switch to logging (sd multiwrite mode) */
    case SdLogger_BeforeLogging:
      if (sdcard1.status == SDCard_MultiWriteIdle) {
        sdlogger.status = SdLogger_Logging;
      }
      break;

      /* Send the stop command and continue with writing status packet */
    case SdLogger_StopLogging:
      if (sdcard1.status != SDCard_MultiWriteIdle) {
        break;
      }
      sdcard_spi_multiwrite_stop(&sdcard1);
      sdlogger.status = SdLogger_WriteStatusPacket;
      break;

      /* Nothing is happening */
    case SdLogger_Idle:
      break;

      /* Busy reading a data block, continuing with sd_logger_packetblock_ready() if not timing out */
    case SdLogger_ReadingBlock:
      sdlogger.timeout_counter++;
      if (sdlogger.timeout_counter > 199) {
        sdlogger.status = SdLogger_Idle;
        sdlogger.timeout_counter = 0;
      }
      break;

    default:
      break;

  }
}

/**
 * @brief Process a command from the ground station
 * This function is called every time a 'setting' is changed at the groundstation,
 * due to this line in the sd_logger.xml: <datalink message="SETTING" fun="sd_logger_command()"/>
 *
 * 'Settings' are modified from the SD logger control application.
 * For example, when the start button is pressed, the sdlogger.cmd setting is set to 1 and this function gets called.
 *
 * If it is not sdlogger.cmd that has changed (case SdLoggerCmd_Nothing), it will check if request_id was changed.
 * In that case, the control application in the ground station requests a log entry.
 */
void sd_logger_command(void)
{

  switch (sdlogger.cmd) {

      /* Start logging command*/
    case SdLoggerCmd_StartLogging:
      if (sdcard1.status != SDCard_Idle) {
        break;
      }
      /* Start at address 1 since 0 is used for the status block/packet */
      sdcard_spi_multiwrite_start(&sdcard1, 0x00000001);
      sdlogger.status = SdLogger_BeforeLogging;
      /* Reserved for unique_id which is set just before writing the block */
      sdlogger.buffer_addr = SD_LOGGER_BLOCK_PREAMBLE_SIZE;
      /* Reset counters since a new log is started */
      sdlogger.packet_count = 0;
      sdlogger.error_count = 0;
      break;


      /* Stop logging command, write last block and fill with zeros if necessary */
    case SdLoggerCmd_StopLogging:
      /* Cannot stop if not logging */
      if (sdlogger.status != SdLogger_Logging) {
        break;
      }

      /* Fill rest of block with trailing zeros */
      for (uint16_t i = sdlogger.buffer_addr; i < (SD_LOGGER_BUFFER_OFFSET + SD_BLOCK_SIZE); i++) {
        sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET + i] = 0x00;
      }
      /* Write the last block */
      sdcard_spi_multiwrite_next(&sdcard1);
      sdlogger.status = SdLogger_StopLogging;
      break;

      /* Request the status packet (first block on the card) */
    case SdLoggerCmd_RequestStatusPacket:
      /* FIXME: If DataAvailable but for the wrong block, and state is not idle, it will still try to read a block. */
      if (sdlogger.status != SdLogger_Idle && sdlogger.status != SdLogger_DataAvailable) {
        break;
      }
      /* No need to read from the SDCard if that block is already available in the buffer */
      if (sdlogger.block_addr == 0x00000000 && sdlogger.status == SdLogger_DataAvailable) {
        sd_logger_send_packet_from_buffer(0);
        break; /* superfluous break */
      } else {
        /* Data not available in buffer, download from SDCard */
        sdcard_spi_read_block(&sdcard1, 0x00000000, &sd_logger_statusblock_ready);
      }
      break;

      /* sdlogger.cmd not changed, but maybe the request_id was set to arrive here. In that case, send the data */
    case SdLoggerCmd_Nothing:
      if (sdlogger.request_id == 0) {
        break;
      }
      /* FIXME: If DataAvailable but for the wrong block, and state is not idle, it will still try to read a block. */
      if (sdlogger.status != SdLogger_Idle && sdlogger.status != SdLogger_DataAvailable) {
        break;
      }
      /* Check if requested data exists in available data. By doing this,
       * the block is read only once for all data points in the block
       * (if they are requested in consequetive order)
       */
      if (sdlogger.status == SdLogger_DataAvailable
          && sdlogger.block_addr == ((sdlogger.request_id - 1) / SD_LOGGER_PACKETS_PER_BLOCK) + 1) {
        /* Send the requested data */
        sd_logger_packetblock_ready();
      } else {
        sdlogger.block_addr = (((sdlogger.request_id - 1) / SD_LOGGER_PACKETS_PER_BLOCK) + 1);
        sdcard_spi_read_block(&sdcard1, sdlogger.block_addr, &sd_logger_packetblock_ready);
        /* Continue with sd_logger_packetblock_ready() in the spi callback */
        sdlogger.status = SdLogger_ReadingBlock;
        sdlogger.timeout_counter = 0;
      }
      break;

    default:
      break;

  }

  /* Always reset, otherwise a command will be executed again when another settings is changed */
  sdlogger.cmd = SdLoggerCmd_Nothing;
}

/**
 * @brief Stop function, nothing is done actually.
 * This is not the same as stopping a recording!
 */
void sd_logger_stop(void)
{

}

/**
 * @brief Send status packet to ground station.
 * Callback which is called when statusblock is available in buffer.
 */
void sd_logger_statusblock_ready(void)
{
  sdlogger.block_addr = 0x00000000;
  sdlogger.status = SdLogger_DataAvailable;
  sd_logger_send_packet_from_buffer(0);
}

/**
 * @brief Send the requested packet to the ground station.
 * Either called immediately if data was already available in the buffer,
 * or called when spi has finished reading the block.
 */
void sd_logger_packetblock_ready(void)
{
  /* If unique id is not matching with the one in the data block,
   * send zeros so that the ground station 'knows' it is corrupted/wrong data */
  if (sdlogger.unique_id != sd_logger_get_uint32(&sdcard1.input_buf[0])) {
    sdlogger.packet.time = 0;
    sdlogger.packet.data_1 = 0;
    sdlogger.packet.data_2 = 0;
    sdlogger.packet.data_3 = 0;
    sdlogger.packet.data_4 = 0;
    sdlogger.packet.data_5 = 0;
    sdlogger.packet.data_6 = 0;
    sdlogger.packet.data_7 = 0;
    sdlogger.packet.data_8 = 0;
    sdlogger.packet.data_9 = 0;
    sdlogger.packet.data_10 = 0;
    sdlogger.packet.data_11 = 0;
    sdlogger.packet.data_12 = 0;
    DOWNLINK_SEND_LOG_DATAPACKET(DefaultChannel, DefaultDevice, &sdlogger.packet.time, &sdlogger.packet.data_1,
                                 &sdlogger.packet.data_2, &sdlogger.packet.data_3, &sdlogger.packet.data_4, &sdlogger.packet.data_5,
                                 &sdlogger.packet.data_6, &sdlogger.packet.data_7, &sdlogger.packet.data_8, &sdlogger.packet.data_9,
                                 &sdlogger.packet.data_10, &sdlogger.packet.data_11, &sdlogger.packet.data_12);
  } else {
    sd_logger_send_packet_from_buffer(((sdlogger.request_id - 1) % SD_LOGGER_PACKETS_PER_BLOCK) * SD_LOGGER_PACKET_SIZE +
                                      SD_LOGGER_BLOCK_PREAMBLE_SIZE);
  }
  /* The buffer now contains data of the block sdlogger.block_addr */
  sdlogger.status = SdLogger_DataAvailable;
}

/**
 * @brief Send log packet message to the ground station.
 * @param buffer_idx Index of the buffer where the log packet starts
 */
void sd_logger_send_packet_from_buffer(uint16_t buffer_idx)
{
  sdlogger.packet.time = sd_logger_get_uint32(&sdcard1.input_buf[buffer_idx + 0]);
  sdlogger.packet.data_1 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx + 4]);
  sdlogger.packet.data_2 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx + 8]);
  sdlogger.packet.data_3 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx + 12]);
  sdlogger.packet.data_4 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx + 16]);
  sdlogger.packet.data_5 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx + 20]);
  sdlogger.packet.data_6 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx + 24]);
  sdlogger.packet.data_7 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx + 28]);
  sdlogger.packet.data_8 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx + 32]);
  sdlogger.packet.data_9 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx + 36]);
  sdlogger.packet.data_10 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx + 40]);
  sdlogger.packet.data_11 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx + 44]);
  sdlogger.packet.data_12 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx + 48]);
  DOWNLINK_SEND_LOG_DATAPACKET(DefaultChannel, DefaultDevice, &sdlogger.packet.time, &sdlogger.packet.data_1,
                               &sdlogger.packet.data_2, &sdlogger.packet.data_3, &sdlogger.packet.data_4, &sdlogger.packet.data_5,
                               &sdlogger.packet.data_6, &sdlogger.packet.data_7, &sdlogger.packet.data_8, &sdlogger.packet.data_9,
                               &sdlogger.packet.data_10, &sdlogger.packet.data_11, &sdlogger.packet.data_12);
}

/* Helper functions */

void sd_logger_int32_to_buffer(const int32_t value, uint8_t *target)
{
  target[0] = value >> 24;
  target[1] = value >> 16;
  target[2] = value >> 8;
  target[3] = value;
}

void sd_logger_uint32_to_buffer(const uint32_t value, uint8_t *target)
{
  target[0] = value >> 24;
  target[1] = value >> 16;
  target[2] = value >> 8;
  target[3] = value;
}

int32_t sd_logger_get_int32(uint8_t *target)
{
  return target[0] << 24 | target[1] << 16 | target[2] << 8 | target[3];
}

uint32_t sd_logger_get_uint32(uint8_t *target)
{
  return target[0] << 24 | target[1] << 16 | target[2] << 8 | target[3];
}
