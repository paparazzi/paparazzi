/*
 * Copyright (C) 2014 Clement Roblot
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file high_speed_logger_direct_memory.c
 * @author Clement Roblot
 * Module used to connect a memory directly to the SPI port
 */


#include "high_speed_logger_direct_memory.h"

#include <string.h>

#include "subsystems/imu.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/uart.h"

#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/telemetry.h"


/******************************************************************/
/*                                                                */
/*                  Config values, go crazy                       */
/*                                                                */
/******************************************************************/
/// size of the block that we read in the memory and send over UART. MAX = 250.
#define READING_BLOCK_SIZE 200
/// if we completly erase the memory at the start of the log.
#define ERASE_MEMORY_AT_START 0
/// size (in bytes) of the values we log.
#define SIZE_OF_LOGGED_VALUES 4
/// size (in characters) of the nameof the logged values.
#define SIZE_OF_VALUES_NAMES 10

/** Skip X values between write.
 * if = 2 we writte a values, then the next two calls to the modules will not
 * add any values to the memory, then the third will add a new values.
 */
#define SKIP_X_CALLS_BETWEEN_VALUES 0

/// nbr of messages you want to log
#define NBR_VALUES_TO_LOG 9

///list of the messages you want to log
uint32_t values_to_log[NBR_VALUES_TO_LOG] = {

  //!!!!!!!!!!!WARNING!!!!!!!!
  //That list must be pointers to the variables
  //you want to log

  (uint32_t) &imu.accel_unscaled.x,
  (uint32_t) &imu.accel_unscaled.y,
  (uint32_t) &imu.accel_unscaled.z,

  (uint32_t) &imu.gyro_unscaled.p,
  (uint32_t) &imu.gyro_unscaled.q,
  (uint32_t) &imu.gyro_unscaled.r,

  (uint32_t) &imu.mag_unscaled.x,
  (uint32_t) &imu.mag_unscaled.y,
  (uint32_t) &imu.mag_unscaled.z
};



///list of the names of the messages you are logging
char **name_of_the_values = (char * [SIZE_OF_VALUES_NAMES])
{
  "acc x",
  "acc y",
  "acc z",
  "gyro p",
  "gyro q",
  "gyro r",
  "mag x",
  "mag y",
  "mag z"
};



/******************************************************************/
/*                                                                */
/*                  Fixed values, do not touch                    */
/*                                                                */
/******************************************************************/
/// nbr of Bytes 0x00 received before the real values when reading the memory
#define MEMORY_READ_LATTENCY 5
/// nbr of MB in the memory
#define TOTAL_MEMORY_SIZE 32
/// the nbr of kilo Bytes left at the end of the memory before stoping the log automaticaly
#define END_OF_MEMORY_THRESHOLD 10


//log start-start_of_values-end flags
///Start sequence written at the begining of a log
uint8_t start_log_sequence[6] = {0xAA, 0x55, 0xFF, 0x00, 0x55, 0xAA};
///Start sequence written at the begining of the values of a log (after the header)
uint8_t start_values_sequence[3] = {0xF0, 0xF0, 0xA5};
///Start sequence indicating we have lost some Bytes due to overflows
uint8_t start_lost_values_sequence[6] = {0x42, 0x0F, 0X42, 0X00, 0XFF, 0xAA};
///Stop sequence indicating we have lost some Bytes due to overflows
uint8_t stop_lost_values_sequence[6] = {0xAA, 0xFF, 0x00, 0x42, 0x0F, 0x42};


///Stop sequence written at the end of the log
uint8_t stop_log_sequence[6] = {0xFF, 0x00, 0x55, 0xAA, 0x00, 0xFF};

//low level buffers used to communicate over SPI
///Buffer used for general comunication over SPI (in buffer)
uint8_t buff[25];   //25 is enough even for the reading of the ID (24 values)
///Buffer used to fetch the values from the memory
uint8_t uart_read_buff[READING_BLOCK_SIZE + MEMORY_READ_LATTENCY] = {0};
///Buffer used for general comunication over SPI (out buffer)
uint8_t msg[10];
///Buffer used for sending value to the memory
uint8_t values_send_buffer[256];

//local buffer used to log values
///First local buffer for the log
uint8_t buffer_values_logged[256];
///Number of Bytes in the first buffer (buffer_values_logged)
uint8_t nbr_values_in_buffer = 0;
///Second local buffer for the log
uint8_t buffer_values_sending[256];
///Number of Bytes in the second buffer (nbr_values_in_buffer_sending)
uint8_t nbr_values_in_buffer_sending = 0;
///Flag defining wich buffer is used
uint8_t buffer_used = 0;

//local status flags
///Flag defining if we are waiting for an answear from the memory when reading values from it
uint8_t wait_answear_from_reading_memory = 0;
///Last status Byte read from the memory
uint8_t memory_status_byte;
///Flag defining if we are sending values through the UART
uint8_t sending_buffer_to_uart = 0;
///Flag defining if we need to keep reading the memory (if the stop sequence is found we stop)
uint8_t relaunch_reading_memory = 0;
///Flag defining if we need to keep reading the memory (if the PC asked for new values)
uint8_t continue_reading_memory = 0;


//memory management variables
///The address at wich we will write next time
uint32_t current_writting_addr = 0x00000000;
///The address of the next block to erase
uint32_t current_unerased_addr = 0x00000000;
///The address at wich we will read next time
uint32_t current_reading_addr = 0x00000000;
///Flag stating if the memory is being used
static volatile bool_t memory_ready = TRUE;
///Structure used for general comunication with the memory
struct spi_transaction memory_transaction;
///Structure used for sending values to the memory
struct spi_transaction memory_send_value_transaction;
///Number of Bytes we have lost due to overflows (reseted each time we can write agin in the buffer)
uint32_t nbr_lost_values = 0;


///Status of the module
uint8_t logging_status_gui;



static void memory_transaction_done_cb(struct spi_transaction *trans);
static void memory_read_status_cb(struct spi_transaction *trans);
static void memory_read_values_cb(struct spi_transaction *trans);


/******************************************************************/
/*                                                                */
/*      Low level function, directly acting with the SPI          */
/*                                                                */
/******************************************************************/

/** @brief Function sending a request for the ID of the memory chip
 *
 * @todo change the cb function to actualy read the value returned
 */
void memory_read_id(void)
{

  memory_ready = FALSE;
  msg[0] = 0x9F;

  memory_transaction.output_buf    = (uint8_t *) msg;
  memory_transaction.output_length = 1;

  memory_transaction.input_buf = (uint8_t *) buff;
  memory_transaction.input_length = 24;

  memory_transaction.after_cb = memory_transaction_done_cb;

  spi_submit(&(HIGH_SPEED_LOGGER_DIRECT_MEMORY_DEVICE), &memory_transaction);
}

/** @brief Function sending a request to set the writte enable flag in the memory
 *
 */
void memory_send_wren(void)
{
  memory_ready = FALSE;
  msg[0] = 0x06;

  memory_transaction.output_buf    = (uint8_t *) msg;
  memory_transaction.output_length = 1;

  memory_transaction.input_buf = NULL;
  memory_transaction.input_length = 0;

  memory_transaction.after_cb = memory_transaction_done_cb;

  spi_submit(&(HIGH_SPEED_LOGGER_DIRECT_MEMORY_DEVICE), &memory_transaction);
}

/** @brief Function sending a request to clear the writte enable flag in the memory
 *
 */
void memory_send_wrdi(void)
{
  memory_ready = FALSE;
  msg[0] = 0x04;

  memory_transaction.output_buf    = (uint8_t *) msg;
  memory_transaction.output_length = 1;

  memory_transaction.input_buf = NULL;
  memory_transaction.input_length = 0;

  memory_transaction.after_cb = memory_transaction_done_cb;

  spi_submit(&(HIGH_SPEED_LOGGER_DIRECT_MEMORY_DEVICE), &memory_transaction);
}

/** @brief Function sending a request to fetch the status Byte of the memory
 *
 */
void memory_read_status_1(void)
{
  memory_ready = FALSE;
  msg[0] = 0x05;

  memory_transaction.output_buf    = (uint8_t *) msg;
  memory_transaction.output_length = 1;

  memory_transaction.input_buf = (uint8_t *) buff;
  memory_transaction.input_length = 4;

  memory_transaction.after_cb = memory_read_status_cb;

  spi_submit(&(HIGH_SPEED_LOGGER_DIRECT_MEMORY_DEVICE), &memory_transaction);
}

/** @brief Callback function decrypting the status Byte of the memory.
 *
 * The resulting value will be setted in the global variable "memory_status_byte".
 *
 */
static void memory_read_status_cb(struct spi_transaction *trans __attribute__((unused)))
{

  memory_ready = TRUE;

  memory_status_byte = buff[1];
  wait_answear_from_reading_memory = 0;
}

/** @brief Function sending a request to erase 4KB of the memory
 *
 * @param mem_addr the address of the block of 4K that you want to erase
 *
 */
void memory_erase_4k(uint32_t mem_addr)
{

  uint8_t *addr = (uint8_t *) &mem_addr;

  memory_ready = FALSE;
  msg[0] = 0x21;
  msg[1] = addr[3];
  msg[2] = addr[2];
  msg[3] = addr[1];
  msg[4] = addr[0];

  memory_transaction.output_buf    = (uint8_t *) msg;
  memory_transaction.output_length = 5;

  memory_transaction.input_buf = NULL;
  memory_transaction.input_length = 0;

  memory_transaction.after_cb = memory_transaction_done_cb;

  spi_submit(&(HIGH_SPEED_LOGGER_DIRECT_MEMORY_DEVICE), &memory_transaction);
}

/** @brief Function sending a request to erase the entire memory
 *
 */
void memory_completly_erase(void)
{

  memory_ready = FALSE;
  msg[0] = 0xC7;

  memory_transaction.output_buf    = (uint8_t *) msg;
  memory_transaction.output_length = 1;

  memory_transaction.input_buf = NULL;
  memory_transaction.input_length = 0;

  memory_transaction.after_cb = memory_transaction_done_cb;

  spi_submit(&(HIGH_SPEED_LOGGER_DIRECT_MEMORY_DEVICE), &memory_transaction);
}

/** @brief Function sending a request to write a buffer of values to the memory
 *
 * @param mem_addr the address at wich the block must beggin to be written
 * @param values pointer to the buffer of value to be written to the memory. This buffer will be copied during this function. It can be reused right after having called this function.
 * @param size the size of the buffer of values
 *
 */
void memory_write_values(uint32_t mem_addr, uint8_t *values, uint8_t size)
{

  uint8_t *addr = (uint8_t *) &mem_addr;
  uint8_t i;

  memory_ready = FALSE;
  values_send_buffer[0] = 0x12;
  values_send_buffer[1] = addr[3];
  values_send_buffer[2] = addr[2];
  values_send_buffer[3] = addr[1];
  values_send_buffer[4] = addr[0];

  for (i = 0; i < size; i++) {
    values_send_buffer[i + 5] = values[i];
  }

  memory_send_value_transaction.output_buf    = (uint8_t *) values_send_buffer;
  memory_send_value_transaction.output_length = 5 + size;

  memory_send_value_transaction.input_buf = NULL;
  memory_send_value_transaction.input_length = 0;

  memory_send_value_transaction.after_cb = memory_transaction_done_cb;

  spi_submit(&(HIGH_SPEED_LOGGER_DIRECT_MEMORY_DEVICE), &memory_send_value_transaction);
}

/** @brief Function sending a request to read some values in memory
 *
 * @param mem_addr the address at wich we want to read
 * @param size the number of Bytes to be read
 *
 */
void memory_read_values(uint32_t mem_addr, uint8_t size)
{

  uint8_t *addr = (uint8_t *) &mem_addr;

  memory_ready = FALSE;
  msg[0] = 0x13;
  msg[1] = addr[3];
  msg[2] = addr[2];
  msg[3] = addr[1];
  msg[4] = addr[0];

  memory_transaction.output_buf    = (uint8_t *) msg;
  memory_transaction.output_length = 5;

  memory_transaction.input_buf = (uint8_t *) uart_read_buff;
  memory_transaction.input_length = size +
                                    MEMORY_READ_LATTENCY; //the first MEMORY_READ_LATTENCY Bytes are lost because of reading to soon

  memory_transaction.after_cb = memory_read_values_cb;

  spi_submit(&(HIGH_SPEED_LOGGER_DIRECT_MEMORY_DEVICE), &memory_transaction);
}

/** @brief Callback function decrypting the read values from the memory
 *
 * The resulting values will be sended to the UART by calling the send_buffer_to_uart function
 *
 */
static void memory_read_values_cb(struct spi_transaction *trans __attribute__((unused)))
{

  uint8_t msg_size = memory_transaction.input_length;

  memory_ready = TRUE;

  if (msg_size) {

    if (is_sequence_in_array(uart_read_buff, msg_size, stop_log_sequence, 6)) { //this is the end of the log

      current_reading_addr = 0;
      relaunch_reading_memory = 0;

    } else { //this is not the end of the log

      relaunch_reading_memory = 1;
    }

    sending_buffer_to_uart = 1;
    send_buffer_to_uart();
  }
}

/** @brief generic allback function for SPI transactions
 *
 * This function will just set a flag telling that the SPI bus can be use now.
 *
 */
static void memory_transaction_done_cb(struct spi_transaction *trans __attribute__((unused)))
{
  memory_ready = TRUE;
}




/******************************************************************/
/*                                                                */
/*     Mid low level functions, abstracting the low SPI layer     */
/*                                                                */
/******************************************************************/

/** @brief Function writting a buffer of values to the memory
 *
 * This function is going to take care of setting the writte enable flag,
 * then will write the values, then will check the status of the write process.
 * Once the writting is done it will return 0.
 * This function must be called multiple times to complet it's work, it is a
 * none blocking function
 *
 * @param mem_addr The address at which we want to write the buffer of values
 * @param values A pointer to the buffer of values we want to write
 * @param size The size of the buffer
 * @return 0 when done, else the status of the state machine (between 1 and 3)
 */
uint8_t ml_write_values_to_memory(uint32_t mem_addr, uint8_t *values, uint8_t size)
{
  static uint8_t ml_write_values_to_memory_status = 0;
  static uint32_t previus_mem_addr = 0;
  static uint8_t *previus_values = NULL;
  static uint8_t previus_size = 0;


  //test if we are stating a new writting cicle
  if ((ml_write_values_to_memory_status == 0) &&
      ((previus_mem_addr != mem_addr) || (previus_values != values) || (previus_size != size))) {

    ml_write_values_to_memory_status = 1;
    previus_mem_addr = mem_addr;
    previus_values = values;
    previus_size = size;
  }

  switch (ml_write_values_to_memory_status) {

    case 0 :
      //waiting for something to do
      break;

    case 1 :
      memory_send_wren();
      ml_write_values_to_memory_status = 2;
      //break;
      //we imediatly send the values after this message.
      //The low level functions are not using the same buffers
      //so we can do it

    case 2 :
      memory_write_values(mem_addr, values, size);
      ml_write_values_to_memory_status = 3;
      wait_answear_from_reading_memory = 1;
      break;

    case 3 :
      memory_read_status_1();  //we wait for the writting to be done
      if ((wait_answear_from_reading_memory) || (memory_status_byte)) {
        ml_write_values_to_memory_status = 3;
      } else { //the writting has been completed
        ml_write_values_to_memory_status = 0;
      }
      break;

    default :
      break;
  }

  return ml_write_values_to_memory_status;
}


/** @brief Function erasing 4KB of the memory
 *
 * This function is going to take care of setting the writte enable flag,
 * then will erase the block, then will check the status of the erase process.
 * Once the erasing is done it will return 0.
 * This function must be called multiple times to complet it's work, it is a
 * none blocking function
 *
 * @param mem_addr The address of the 4KB block we want to erase
 * @return 0 when done, else the status of the state machine (between 1 and 2)
 */
uint8_t ml_erase_4k_on_memory(uint32_t mem_addr)
{

  static uint8_t ml_erase_4k_on_memory_status = 0;

  switch (ml_erase_4k_on_memory_status) {

    case 0 :
      memory_send_wren();
      ml_erase_4k_on_memory_status = 1;
      break;

    case 1 :
      memory_erase_4k(mem_addr);
      ml_erase_4k_on_memory_status = 2;
      wait_answear_from_reading_memory = 1;
      break;

    case 2 :
      memory_read_status_1();  //we wait for the writting to be done
      if ((wait_answear_from_reading_memory) || (memory_status_byte)) {
        ml_erase_4k_on_memory_status = 2;
      } else { //the erasing have been completed
        ml_erase_4k_on_memory_status = 0;
      }
      break;

    default :
      break;
  }

  return ml_erase_4k_on_memory_status;
}


/** @brief Function erasing the entire memory
 *
 * This function is going to take care of setting the writte enable flag,
 * then will erase the memory, then will check the status of the erase process.
 * Once the erasing is done it will return 0.
 * This function must be called multiple times to complet it's work, it is a
 * none blocking function
 *
 * @return 0 when done, else the status of the state machine (between 1 and 2)
 */
uint8_t ml_erase_completely_memory(void)
{

  static uint8_t ml_erase_memory_status = 0;

  switch (ml_erase_memory_status) {

    case 0 :
      memory_send_wren();
      ml_erase_memory_status = 1;
      break;

    case 1 :
      memory_completly_erase();
      ml_erase_memory_status = 2;
      wait_answear_from_reading_memory = 1;
      break;

    case 2 :
      memory_read_status_1();  //we wait for the writting to be done
      if ((wait_answear_from_reading_memory) || (memory_status_byte)) {
        ml_erase_memory_status = 2;
      } else { //the erasing have been completed
        ml_erase_memory_status = 0;
      }
      break;

    default : break;
  }

  return ml_erase_memory_status;
}


/** @brief Function continuing the reading of the current log in memory
 *
 * This function will start the reading of a new block of the memory,
 * the size of this block is defined by READING_BLOCK_SIZE.
 *
 */
void ml_read_log_in_memory(void)
{
  memory_read_values(current_reading_addr, READING_BLOCK_SIZE);
  current_reading_addr += READING_BLOCK_SIZE;
}





/******************************************************************/
/*                                                                */
/*      Mid high level function : memory management               */
/*                                                                */
/******************************************************************/

/** @brief Function adding a buffer of values to the memory
 *
 * This function is going to take care of writting a buffer of values
 * into the memory. It starts at the address 0x00000000 and will writte
 * after that. It keeps the last written address in memory and will
 * erase the blocks if needed and configured (ERASE_MEMORY_AT_START) before
 * programming the values into the blocks.
 * This function is non blocking, it must be called multiple times to
 * complet it's work.
 *
 * @param values the buffer of values to writte to the memory
 * @param size the size of the buffer of values to writte
 * @return 0 when done, else return 1
 */
uint8_t append_values_to_memory(uint8_t *values, uint8_t size)
{

  static uint8_t append_to_memory_status = 0;
  static uint8_t index_value_unwritten = 0;
  uint8_t return_code = 1;
  uint8_t size_data_left_to_write;
  uint8_t wait_for_SPI = 0;

  uint32_t size_used_in_current_page, size_left_current_page, size_to_write;

  while (!wait_for_SPI) {

    switch (append_to_memory_status) {

      case 0 :
        if (!ERASE_MEMORY_AT_START) {
          if ((current_unerased_addr - current_writting_addr) > size) { //we got enough cleared space to write the value

            append_to_memory_status = 1;
          } else {

            if (!ml_erase_4k_on_memory(current_unerased_addr)) {

              current_unerased_addr += 4096;   //65536
            } else {

              wait_for_SPI = 1;
            }
          }
        } else {

          append_to_memory_status = 1;
        }
        break;


      case 1 :
        //we start writting the values
        //need to fit the values in pages of 4K

        //256 bytes per pages
        size_used_in_current_page = (current_writting_addr & 0x000000FF);
        size_left_current_page = 0x00000100 - size_used_in_current_page;

        size_data_left_to_write = size - index_value_unwritten;
        if (size_left_current_page > size_data_left_to_write) { size_to_write = size_data_left_to_write; }
        else { size_to_write = size_left_current_page; }

        if (size_to_write > 250) { size_to_write = 250; } //protection against overflows in the lower levels

        if (size_to_write > 0) {

          if (!ml_write_values_to_memory(current_writting_addr, &values[index_value_unwritten], size_to_write)) {

            index_value_unwritten += size_to_write;
            current_writting_addr += size_to_write;

            if (index_value_unwritten == size) {

              append_to_memory_status = 0;
              index_value_unwritten = 0;
              return_code = 0;
            }

            //test is we are close to the end of the memory
            if (((TOTAL_MEMORY_SIZE << 20) - current_writting_addr) < (END_OF_MEMORY_THRESHOLD << 10)) {
              //if we got less than END_OF_MEMORY_THRESHOLD kilo Bytes of mem left

              logging_status_gui = 3; //we stop the logging
            }


          }

          wait_for_SPI = 1;
        }
        break;

      default:
        break;
    }
  }

  return return_code;
}



/** @brief Function sending the read values from the memory to te UART
 *
 * This function will send the buffer of values read back to the UART
 * and will launch an other blokc to be read if needed.
 *
 */
void send_buffer_to_uart(void)
{

  uint8_t msg_size = memory_transaction.input_length;
  static uint8_t i = MEMORY_READ_LATTENCY;

  if (sending_buffer_to_uart) {

    while (uart_check_free_space(&HS_LOG_UART, 1)) {

      if (i >= msg_size) {

        sending_buffer_to_uart = 0;
        i = MEMORY_READ_LATTENCY;
        if (relaunch_reading_memory && continue_reading_memory) {

          continue_reading_memory = 0;
          ml_read_log_in_memory();
        }
        break;
      }
      uart_transmit(&HS_LOG_UART, uart_read_buff[i]);
      i++;
    }

  } else {

    i = MEMORY_READ_LATTENCY;
    if (relaunch_reading_memory && continue_reading_memory) {
      continue_reading_memory = 0;
      ml_read_log_in_memory();
    }
  }
}



/** @brief Function sending a buffer ot the memory
 *
 * This function is just a verification that the buffer is
 * not empty and that we are not asking for writting an empty buffer.
 * It is simply an abstraction of the append_values_to_memory function.
 *
 * @param buffer the buffer of values to writte to the memory
 * @param size the size of the buffer of values to writte
 * @return 0 when done, else return 1
 */
uint8_t send_buffer_to_memory(uint8_t *buffer, uint8_t *size)
{

  uint8_t return_code = 1;

  if (*size > 0) {
    if (!append_values_to_memory(buffer, *size)) {
      *size = 0;
      return_code = 0;
    }
  } else {
    return_code = 0;
  }

  return return_code;
}




/******************************************************************/
/*                                                                */
/*      High level function : local buffer management             */
/*                                                                */
/******************************************************************/

/** @brief Function adding a Byte to the local buffer
 *
 * This function will just add the value sended to the local buffer.
 * The buffer will then be automaticaly written to the memory when
 * possible.
 *
 * @param value the value to add to the buffer
 *
 * @todo Add a protection against overflows
 */
void add_byte_to_buffer(uint8_t value)
{
  if (buffer_used) {

    if (nbr_values_in_buffer_sending < 0xFF) {
      buffer_values_sending[nbr_values_in_buffer_sending] = value;
      nbr_values_in_buffer_sending++;
    } else {
      nbr_lost_values++;
    }

  } else {

    if (nbr_values_in_buffer < 0xFF) {
      buffer_values_logged[nbr_values_in_buffer] = value;
      nbr_values_in_buffer++;
    } else {
      nbr_lost_values++;
    }
  }
}

/** @brief Function adding an array to the local buffer
 *
 * This function will just add the values sended to the local buffer.
 * The buffer will then be automaticaly written to the memory when
 * possible.
 *
 * @param array the array to add to the buffer
 * @param size the size of the array to add to the buffer
 *
 */
void add_array_to_buffer(uint8_t *array, uint8_t size)
{
  uint8_t i;

  for (i = 0; i < size; i++) {
    add_byte_to_buffer(array[i]);
  }
}

/** @brief Function sending the buffer to the memory when possible
 *
 * This function will send a local buffer to the memory when possible.
 *
 * @todo We don't need two buffers as far as they are imadiatly copied when sended to the memory
 */
uint8_t run_memory_management(void)
{
  uint8_t return_code = 1;
  uint8_t *log_value_tmp;

  if (buffer_used) {

    if (!send_buffer_to_memory(buffer_values_logged, &nbr_values_in_buffer)) {

      buffer_used = 0;

      if (nbr_lost_values) {

        add_array_to_buffer(start_lost_values_sequence, 6);

        log_value_tmp = (uint8_t *) &nbr_lost_values;
        add_byte_to_buffer(log_value_tmp[0]);
        add_byte_to_buffer(log_value_tmp[1]);
        add_byte_to_buffer(log_value_tmp[2]);
        add_byte_to_buffer(log_value_tmp[3]);

        add_array_to_buffer(stop_lost_values_sequence, 6);
        nbr_lost_values = 0;
      }

      return_code = 0;
    }
  } else {

    if (!send_buffer_to_memory(buffer_values_sending, &nbr_values_in_buffer_sending)) {

      buffer_used = 1;

      if (nbr_lost_values) {

        add_array_to_buffer(start_lost_values_sequence, 6);

        log_value_tmp = (uint8_t *) &nbr_lost_values;
        add_byte_to_buffer(log_value_tmp[0]);
        add_byte_to_buffer(log_value_tmp[1]);
        add_byte_to_buffer(log_value_tmp[2]);
        add_byte_to_buffer(log_value_tmp[3]);

        add_array_to_buffer(stop_lost_values_sequence, 6);
        nbr_lost_values = 0;
      }

      return_code = 0;
    }
  }

  return return_code;
}



/** @brief Function returning true if the two local buffers are empty
 *
 */
uint8_t are_buffers_empty(void)
{
  uint8_t return_value = 0;

  if ((nbr_values_in_buffer == 0) && (nbr_values_in_buffer_sending == 0)) {
    return_value = 1;
  }

  return return_value;
}




/******************************************************************/
/*                                                                */
/*      High level function : log management                    */
/*                                                                */
/******************************************************************/

/** @brief Function starting a new log
 *
 * This function will start a new log by erasing he memory if configured (ERASE_MEMORY_AT_START)
 * and then will send the starting tags and the name of the messages.
 *
 * This function is none blokcing, it need to be called multiples times to complete
 * it's work. It will return 0 when the work have been done.

 * @return 0 when done, 1 in other case.
 *
 * @todo Add the size of the values sended (in Bytes : SIZE_OF_LOGGED_VALUES).
 * @todo Add a space to writte the total size of that log (for the progress bar in the downloading application)
 */
uint8_t start_new_log(void)
{

  static uint8_t start_log_status = 0;
  uint8_t return_code = 1;

  char msg_names[(SIZE_OF_VALUES_NAMES + 1)*NBR_VALUES_TO_LOG] = "";
  uint8_t i;

  for (i = 0; i < NBR_VALUES_TO_LOG; i++) {

    strcat(msg_names, name_of_the_values[i]);
    if (i != (NBR_VALUES_TO_LOG - 1)) { strcat(msg_names, ";"); } //we don't put a ';' at the end of the list
  }

  switch (start_log_status) {

    case 0 :
      current_writting_addr = 0x00000000; //restart the writting at the begining of the memory
      current_unerased_addr = 0x00000000;

      if (ERASE_MEMORY_AT_START) {

        if (!ml_erase_completely_memory()) {

          start_log_status = 1;
        }
      } else {

        start_log_status = 1;
      }
      break;


    case 1 :
      add_array_to_buffer(start_log_sequence, 6);
      add_byte_to_buffer(SIZE_OF_LOGGED_VALUES);
      start_log_status = 2;
      break;

    case 2 :
      add_array_to_buffer((uint8_t *)msg_names, (SIZE_OF_VALUES_NAMES + 1)*NBR_VALUES_TO_LOG);
      start_log_status = 3;
      break;

    case 3 :
      add_array_to_buffer(start_values_sequence, 3);
      start_log_status = 0;
      return_code = 0;
      break;

    default :
      break;
  }

  return return_code;
}


/** @brief Function adding the configured messages to the buffers of the values to be written in memory
 *
 * This function will fetch the values of the variables setted in the array values_to_log, and
 * send them to the local buffer to be written in memory
 *
 */
void add_values_to_buffer(void)
{
  uint8_t *log_value_tmp;
  uint8_t i, j;

  for (i = 0; i < NBR_VALUES_TO_LOG; i++) {

    log_value_tmp = (uint8_t *) values_to_log[i];

    for (j = 0; j < SIZE_OF_LOGGED_VALUES; j++) {

      add_byte_to_buffer(log_value_tmp[j]);
    }
  }
}


/** @brief Funcion called to add the values to log to the buffer with a frequency divider in order to not overflow the buffers
 *
 */
void run_logger(void)
{
  static uint8_t i = 0;
  //useless variable. It simply remove a warning when SKIP_X_CALLS_BETWEEN_VALUES=0 and the test is always true
  uint8_t limit = SKIP_X_CALLS_BETWEEN_VALUES;

  if (i >= limit) {  //3 with erase memory, 0 WITHOUT SKIP_X_CALLS_BETWEEN_VALUES
    add_values_to_buffer();
    i = 0;
  }
  i++;
}


/** @brief Function ending the current log
 *
 * This function will simply write the ending tag of the log.
 *
 * This function is none blocking and need to be called multiple times
 * to complete it's work. It will return 0 when the work is completed.
 *
 * @return 0 when done, 1 in other cases.
 *
 */
uint8_t end_log(void)
{
  uint8_t return_value = 1;

  if (are_buffers_empty()) { //if the buffers are empty to prevent from writting during a buffer overflow
    add_array_to_buffer(stop_log_sequence, 6);
    return_value = 0;
  }

  return return_value;
}




/******************************************************************/
/*                                                                */
/*                      Module function                           */
/*                                                                */
/******************************************************************/


/** @brief Function initialisating the module
 *
 */
void high_speed_logger_direct_memory_init(void)
{
  //init the SPI to communicat with the memory
  memory_transaction.select        = SPISelectUnselect;
  memory_transaction.cpol          = SPICpolIdleHigh;
  memory_transaction.cpha          = SPICphaEdge2;
  memory_transaction.dss           = SPIDss8bit;
  memory_transaction.bitorder      = SPIMSBFirst;
  memory_transaction.cdiv          = SPIDiv64;
  memory_transaction.slave_idx     = HIGH_SPEED_LOGGER_DIRECT_MEMORY_SLAVE_NUMBER;
  memory_transaction.select        = SPISelectUnselect;
  memory_transaction.output_buf    = NULL;
  memory_transaction.output_length = 0;
  memory_transaction.input_buf     = NULL;
  memory_transaction.input_length  = 0;
  memory_transaction.after_cb      = memory_transaction_done_cb;

  spi_submit(&(HIGH_SPEED_LOGGER_DIRECT_MEMORY_DEVICE), &memory_transaction);


  memory_send_value_transaction.select        = SPISelectUnselect;
  memory_send_value_transaction.cpol          = SPICpolIdleHigh;
  memory_send_value_transaction.cpha          = SPICphaEdge2;
  memory_send_value_transaction.dss           = SPIDss8bit;
  memory_send_value_transaction.bitorder      = SPIMSBFirst;
  memory_send_value_transaction.cdiv          = SPIDiv64;
  memory_send_value_transaction.slave_idx     = HIGH_SPEED_LOGGER_DIRECT_MEMORY_SLAVE_NUMBER;
  memory_send_value_transaction.select        = SPISelectUnselect;
  memory_send_value_transaction.output_buf    = NULL;
  memory_send_value_transaction.output_length = 0;
  memory_send_value_transaction.input_buf     = NULL;
  memory_send_value_transaction.input_length  = 0;
  memory_send_value_transaction.after_cb      = memory_transaction_done_cb;



  //init the UART to send the values back to the computer
  uart_periph_init(&HS_LOG_UART);
  uart_periph_set_bits_stop_parity(&HS_LOG_UART, 8, 1, 0);
}



/** @brief Main function of the module. Will manage the log.
 *
 */
void high_speed_logger_direct_memory_periodic(void)
{
  uint8_t uart_received;

  //UART part (communication with the computer to dump the memory)
  while (uart_char_available(&HS_LOG_UART)) {

    uart_received = uart_getch(&HS_LOG_UART);

    if (uart_received == 'A') {
      continue_reading_memory = 1;
      ml_read_log_in_memory();

    } else if (uart_received == 'B') {
      continue_reading_memory = 1;
    }
  }


  //SPI part (to log the values on the memory when in flight)
  if (memory_ready) {

    switch (logging_status_gui) {

      case 0 :
        //idle state, nothing to do
        break;

      case 1 :
        //start a new log
        if (!start_new_log()) { logging_status_gui = 0; }
        break;

      case 2 :
        //logging values
        run_logger();
        break;

      case 3 :
        //finishing the log
        if (!end_log()) { logging_status_gui = 0; }
        break;

      default :
        break;
    }

    run_memory_management();
  }

  //this function regulats itself, we have to call it at every iteration of the module
  send_buffer_to_uart();
}



/******************************************************************/
/*                                                                */
/*                      Other functions                           */
/*                                                                */
/******************************************************************/

/** @brief Function testing if a sequence is in a buffer of values
 *
 * This function is usefull to detect the tag indicating the end of the
 * log in order to stop reading. This function will remember the previus
 * values and as such can detect tags even when cutted. We can detect the
 * first half of the tag at the end of one buffer and detect the other half
 * at the begining of the folowing buffer.
 *
 * @param array the buffer in wich we are searching for the sequence
 * @param array_size the size of the buffer in wich we are searching
 * @param sequence the tag we are searching for
 * @param sequence_size the size of the tag we are searching for
 *
 */
uint8_t is_sequence_in_array(uint8_t *array, uint8_t array_size, uint8_t *sequence, uint8_t sequence_size)
{
  uint8_t i = MEMORY_READ_LATTENCY;
  static uint8_t current_sequence_id = 0;

  for (i = MEMORY_READ_LATTENCY; i < array_size; i++) {

    if (array[i] == sequence[current_sequence_id]) {
      current_sequence_id++;
      if (current_sequence_id >= sequence_size) {
        current_sequence_id = 0;
        return 1;
      }
    } else {
      current_sequence_id = 0;
    }
  }

  return 0;
}


/** @brief Function managing the interface with the user
 *
 */
void high_speed_logger_direct_memory_handler(uint8_t val)
{
  logging_status_gui = val;
}
