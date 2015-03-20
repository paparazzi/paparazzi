/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file modules/loggers/direct_memory_logger.c
 * @brief Write logs directly to flash memory chips
 */

#include "direct_memory_logger.h"
#include "mcu_periph/uart.h"
#include "subsystems/imu.h"
#include "stabilization.h"

struct DirectMemoryLogger dml;
static void direct_memory_spi_cb(struct spi_transaction *trans);
static int32_t seq_in_array(uint8_t *array, uint16_t array_size, uint8_t *sequence, uint16_t sequence_size);

// Different sequences
static uint8_t start_log_sequence[6] = {0xAA, 0x55, 0xFF, 0x00, 0x55, 0xAA};
static uint8_t stop_log_sequence[6] = {0xFF, 0x00, 0x55, 0xAA, 0x10, 0xFF};


// Logging struct
struct LogStruct {
  uint32_t counter;
  int32_t accel_z;
  int32_t gyro_p;
  int32_t gyro_q;
  int32_t gyro_r;
  int32_t thrust;
} __attribute__((packed));
static struct LogStruct log_struct;
static uint32_t dm_counter = 0;

static int32_t seq_in_array(uint8_t *array, uint16_t array_size, uint8_t *sequence, uint16_t sequence_size)
{
  uint16_t i;
  static uint16_t current_sequence_id = 0;
  static uint16_t count_ff = 0;

  for (i = 0; i < array_size; i++) {

    // Detect stop sequence
    if (array[i] == sequence[current_sequence_id]) {
      current_sequence_id++;
      if (current_sequence_id >= sequence_size) {
        count_ff = 0;
        current_sequence_id = 0;
        return i;
      }
    } else {
      current_sequence_id = 0;
    }

    // Detect ff sequence
    if (array[i] == 0xFF) {
      count_ff++;

      if (count_ff >= 1000) {
        count_ff = 0;
        current_sequence_id = 0;
        return i;
      }
    } else {
      count_ff = 0;
    }
  }

  return -1;
}

void direct_memory_logger_init(void)
{
  dml.status = DML_INIT;

  // Initialize the sst chip
  sst25vfxxxx_init(&dml.sst, &(DM_LOG_SPI_DEV), DM_LOG_SPI_SLAVE_IDX, &direct_memory_spi_cb);
}

void direct_memory_logger_periodic(void)
{
  int32_t seq_idx;
  uint16_t i, end_idx;

  // Switch the different statusses
  switch (dml.status) {
    case DML_IDLE:
      // Do nothing
      break;

      // Stopping
    case DML_STOP:
      if (dml.sst.status != SST25VFXXXX_IDLE) {
        break;
      }

      dml.status = DML_IDLE;
      dml.write_addr = dml.sst.flash_addr + 6;
      sst25vfxxxx_write(&dml.sst, stop_log_sequence, 6);
      break;

      // Logging
    case DML_START:
      dm_counter = 0;
      dml.status = DML_LOGGING;
    case DML_LOGGING:
      // Check if too slow TODO fix error
      dm_counter++;
      if (dml.sst.status != SST25VFXXXX_IDLE) {
        break;
      }

      // Set the log values
      log_struct.counter++;
      log_struct.accel_z   = imu.accel.z;
      log_struct.gyro_p    = imu.gyro.p;
      log_struct.gyro_q    = imu.gyro.q;
      log_struct.gyro_r    = imu.gyro.r;
      log_struct.thrust    = stabilization_cmd[COMMAND_THRUST];

      sst25vfxxxx_write(&dml.sst, (uint8_t *) &log_struct, sizeof(struct LogStruct));
      break;

      // Reading
    case DML_READ:
      dml.status = DML_READING;
    case DML_READING:

      if (DM_LOG_UART.tx_running || dml.sst.status != SST25VFXXXX_IDLE) {
        break;
      }

      // Detect end sequence
      seq_idx = seq_in_array(&(dml.buffer[5]), DML_BUF_SIZE - 5, stop_log_sequence, 6);
      if (seq_idx < 0) {
        end_idx = DML_BUF_SIZE;
      } else {
        end_idx = seq_idx + 5;
        dml.status = DML_IDLE;
      }

      for (i = 5; i < end_idx; i++) {
        uart_transmit(&DM_LOG_UART, dml.buffer[i]);
      }

      // Read next bytes
      dml.sst.flash_addr += end_idx - 5;
      if (seq_idx < 0) {
        sst25vfxxxx_read(&dml.sst, dml.buffer, DML_BUF_SIZE - 5);
      }
      break;

    default:
      if (dml.sst.status == SST25VFXXXX_IDLE) {
        dml.status = DML_IDLE;
      }
      break;
  }
}

void direct_memory_logger_set(uint8_t val)
{
  // First handle stopping while logging
  if (dml.status == DML_LOGGING && val == DML_STOP) {
    dml.status = DML_STOP;
    return;
  }

  // Handle only while idle
  if (dml.status != DML_IDLE) {
    return;
  }

  // Handle all the statuses
  dml.status = val;
  switch (dml.status) {
    case DML_ERASE:
      dml.sst.flash_addr = 0x0;
      dml.write_addr = 0x0;
      sst25vfxxxx_chip_erase(&dml.sst);
      break;
    case DML_START:
      dml.sst.flash_addr = dml.write_addr;
      sst25vfxxxx_write(&dml.sst, start_log_sequence, 6);
      break;
    case DML_READ:
      dml.sst.flash_addr = 0x0;
      sst25vfxxxx_read(&dml.sst, dml.buffer, DML_BUF_SIZE - 5);
      break;
    default:
      break;
  }
}

static void direct_memory_spi_cb(__attribute__((unused)) struct spi_transaction *trans)
{
  sst25vfxxxx_after_cb(&dml.sst);
}
