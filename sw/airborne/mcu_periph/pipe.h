/*
 * Copyright (C) 2018 Kirk Scheper <kirkscheper@gmail.com>
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

/** \file mcu_periph/pipe.h
 *  \brief arch independent PIPE API
 *
 */

#ifndef MCU_PERIPH_PIPE_H
#define MCU_PERIPH_PIPE_H

#include "std.h"
#include "mcu_periph/pipe_arch.h"
#include "pprzlink/pprzlink_device.h"

struct pipe_periph {
  /** Receive buffer */
  uint8_t rx_buf[PIPE_RX_BUFFER_SIZE];
  uint16_t rx_insert_idx;
  uint16_t rx_extract_idx;
  /** Transmit buffer */
  uint8_t tx_buf[PIPE_TX_BUFFER_SIZE];
  uint16_t tx_insert_idx;
  /** PIPE file descriptor */
  int fd_read;
  int fd_write;
  /** Generic device interface */
  struct link_device device;
};

extern void     pipe_arch_init(void);
extern void     pipe_arch_periph_init(struct pipe_periph *p, char *read_name, char* write_name);
extern void     pipe_periph_init(struct pipe_periph *p, char *name, char* write_name);
extern int      pipe_char_available(struct pipe_periph *p);
extern uint8_t  pipe_getch(struct pipe_periph *p);
extern void     pipe_receive(struct pipe_periph *p);
extern void     pipe_send_message(struct pipe_periph *p, long fd);
extern void     pipe_send_raw(struct pipe_periph *p, long fd, uint8_t *buffer, uint16_t size);
extern int      pipe_check_free_space(struct pipe_periph *p, long *fd, uint16_t len);
extern void     pipe_put_byte(struct pipe_periph *p, long fd, uint8_t data);
extern void     pipe_put_buffer(struct pipe_periph *p, long fd, const uint8_t *data,uint16_t len);

#if defined(USE_PIPE0_WRITER) || defined(USE_PIPE0_READER)
extern struct pipe_periph pipe0;

#ifndef USE_PIPE0_WRITER
#define USE_PIPE0_WRITER NULL
#endif

#ifndef USE_PIPE0_READER
#define USE_PIPE0_READER NULL
#endif

#define PIPE0Init() pipe_periph_init(&pipe0, STRINGIFY(USE_PIPE0_READER), STRINGIFY(USE_PIPE0_WRITER))
#endif // USE_PIPE0

#if defined(USE_PIPE1_WRITER) || defined(USE_PIPE1_READER)
extern struct pipe_periph pipe1;

#ifndef USE_PIPE1_WRITER
#define USE_PIPE1_WRITER NULL
#endif

#ifndef USE_PIPE1_READER
#define USE_PIPE1_READER NULL
#endif

#define PIPE1Init() pipe_periph_init(&pipe1, STRINGIFY(USE_PIPE1_READER), STRINGIFY(USE_PIPE1_WRITER))
#endif // USE_PIPE1

#if defined(USE_PIPE2_WRITER) || defined(USE_PIPE2_READER)
extern struct pipe_periph pipe2;

#ifndef USE_PIPE2_WRITER
#define USE_PIPE2_WRITER NULL
#endif

#ifndef USE_PIPE2_READER
#define USE_PIPE2_READER NULL
#endif

#define PIPE2Init() pipe_periph_init(&pipe2, STRINGIFY(USE_PIPE2_READER), STRINGIFY(USE_PIPE2_WRITER))
#endif // USE_PIPE2

#endif /* MCU_PERIPH_PIPE_H */
