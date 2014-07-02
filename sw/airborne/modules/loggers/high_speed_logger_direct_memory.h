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
 * @file high_speed_logger_direct_memory.h
 * @author Clement Roblot
 * Module used to connect a memory directly to the SPI port
 */

#ifndef HIGH_SPEED_LOGGER_DIRECT_MEMORY_H_
#define HIGH_SPEED_LOGGER_DIRECT_MEMORY_H_

#include "std.h"


//Low level functions, directly acting with the SPI
extern void memory_read_id(void);
extern void memory_send_wren(void);
extern void memory_send_wrdi(void);
extern void memory_read_status_1(void);
extern void memory_read_values(uint32_t mem_addr, uint8_t size);
extern void memory_write_values(uint32_t mem_addr, uint8_t *values, uint8_t size);
extern void memory_completly_erase(void);
extern void memory_erase_4k(uint32_t mem_addr);

//Mid low level functions, abstracting the low SPI layer
extern uint8_t ml_write_values_to_memory(uint32_t mem_addr, uint8_t *values, uint8_t size);
extern uint8_t ml_erase_4k_on_memory(uint32_t mem_addr);
extern uint8_t ml_erase_completely_memory(void);
extern void ml_read_log_in_memory(void);

//Mid high level function : memory management
extern uint8_t append_values_to_memory(uint8_t *values, uint8_t size);
extern uint8_t send_buffer_to_memory(uint8_t *buffer, uint8_t *size);
extern void send_buffer_to_uart(void);

//High level function : local buffer management
extern void add_byte_to_buffer(uint8_t value);
extern void add_array_to_buffer(uint8_t *array, uint8_t size);
extern uint8_t run_memory_management(void);
extern uint8_t are_buffers_empty(void);

//High level function : log commands
extern uint8_t start_new_log(void);
extern void add_values_to_buffer(void);
extern void run_logger(void);
extern uint8_t end_log(void);

//Module function
extern void high_speed_logger_direct_memory_init(void);
extern void high_speed_logger_direct_memory_periodic(void);

//Other functions
extern uint8_t is_sequence_in_array(uint8_t *array, uint8_t array_size, uint8_t *sequence, uint8_t sequence_size);

extern void high_speed_logger_direct_memory_handler(uint8_t val);

extern uint8_t logging_status_gui;

#endif /* HIGH_SPEED_LOGGER_DIRECT_MEMORY_H_ */
