/*
 * Copyright (C) 2018 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file arch/linux/mcu_periph/pipe_arch.c
 * linux named pipe handling
 */

#include "mcu_periph/pipe.h"
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <sys/select.h>

// FIFO
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "rt_priority.h"

#ifndef PIPE_THREAD_PRIO
#define PIPE_THREAD_PRIO 10
#endif

static void *pipe_thread(void *data __attribute__((unused)));
static pthread_mutex_t pipe_mutex = PTHREAD_MUTEX_INITIALIZER;

void pipe_arch_init(void)
{
  pthread_mutex_init(&pipe_mutex, NULL);

#if defined(USE_PIPE0_WRITER) || defined(USE_PIPE0_READER)
  PIPE0Init();
#endif
#if defined(USE_PIPE1_WRITER) || defined(USE_PIPE1_READER)
  PIPE1Init();
#endif
#if defined(USE_PIPE2_WRITER) || defined(USE_PIPE2_READER)
  PIPE2Init();
#endif

  pthread_t tid;
  if (pthread_create(&tid, NULL, pipe_thread, NULL) != 0) {
    fprintf(stderr, "pipe_arch_init: Could not create PIPE reading thread.\n");
    return;
  }
#ifndef __APPLE__
  pthread_setname_np(tid, "pipe");
#endif
}

/**
 * Initialize the PIPE peripheral.
 * Allocate UdpSocket struct and create and bind the PIPE socket.
 */
void pipe_arch_periph_init(struct pipe_periph *p, char *read_name, char* write_name)
{
  if(read_name != NULL)
  {
    if( access( read_name, F_OK ) == -1 ) {
      mkfifo(read_name, 0666);
    }
    p->fd_read = open(read_name, O_RDWR | O_NONBLOCK);
  } else {
    p->fd_read = -1;
  }

  if(write_name != NULL)
  {
    if( access( write_name, F_OK ) == -1 ) {
      mkfifo(write_name, 0666);
    }
    p->fd_write = open(write_name, O_RDWR | O_NONBLOCK);
  } else {
    p->fd_write = -1;
  }
}

/**
 * Get number of bytes available in receive buffer.
 * @param p pointer to PIPE peripheral
 * @return number of bytes available in receive buffer
 */
int pipe_char_available(struct pipe_periph *p)
{
  pthread_mutex_lock(&pipe_mutex);
  int available = p->rx_insert_idx - p->rx_extract_idx;
  if (available < 0) {
    available += PIPE_RX_BUFFER_SIZE;
  }
  pthread_mutex_unlock(&pipe_mutex);
  return available;
}

/**
 * Get the last character from the receive buffer.
 * @param p pointer to PIPE peripheral
 * @return last byte
 */
uint8_t pipe_getch(struct pipe_periph *p)
{
  pthread_mutex_lock(&pipe_mutex);
  uint8_t ret = p->rx_buf[p->rx_extract_idx];
  p->rx_extract_idx = (p->rx_extract_idx + 1) % PIPE_RX_BUFFER_SIZE;
  pthread_mutex_unlock(&pipe_mutex);
  return ret;
}

/**
 * Read bytes from PIPE
 */
void pipe_receive(struct pipe_periph *p)
{
  if (p == NULL) { return; }
  if (p->fd_read < 0) { return; }

  int16_t i;
  int16_t available = PIPE_RX_BUFFER_SIZE - pipe_char_available(p);
  uint8_t buf[PIPE_RX_BUFFER_SIZE];

  if (available <= 0) {
    return;  // No space
  }

  ssize_t bytes_read = read(p->fd_read, buf, available);

  pthread_mutex_lock(&pipe_mutex);
  if (bytes_read > 0) {
    for (i = 0; i < bytes_read; i++) {
      p->rx_buf[p->rx_insert_idx] = buf[i];
      p->rx_insert_idx = (p->rx_insert_idx + 1) % PIPE_RX_BUFFER_SIZE;
    }
  }
  pthread_mutex_unlock(&pipe_mutex);
}

/**
 * Send a message
 */
void pipe_send_message(struct pipe_periph *p, long fd __attribute__((unused)))
{
  if (p == NULL) { return; }
  if (p->fd_write < 0) { return; }

  if (p->tx_insert_idx > 0) {
    ssize_t bytes_sent = write(p->fd_write, p->tx_buf, p->tx_insert_idx);
    if (bytes_sent != p->tx_insert_idx) {
      if (bytes_sent < 0) {
        fprintf(stderr, "pipe_send_message failed\n");
      } else {
        fprintf(stderr, "pipe_send_message: only sent %d bytes instead of %d\n",
                (int)bytes_sent, p->tx_insert_idx);
      }
    }
    p->tx_insert_idx = 0;
  }
}

/**
 * Send a packet from another buffer
 */
void pipe_send_raw(struct pipe_periph *p, long fd __attribute__((unused)), uint8_t *buffer, uint16_t size)
{
  if (p == NULL) { return; }
  if (p->fd_write < 0) { return; }

  ssize_t test __attribute__((unused)) = write(p->fd_write, buffer, size);
}

/**
 * check for new pipe packets to receive.
 */
static void *pipe_thread(void *data __attribute__((unused)))
{
  get_rt_prio(PIPE_THREAD_PRIO);

  /* file descriptor list */
  fd_set fds_master;
  /* maximum file descriptor number */
  int fdmax = 0;

  /* clear the fd list */
  FD_ZERO(&fds_master);
  /* add used file descriptors */
  int fd __attribute__((unused));
#ifdef USE_PIPE0_READER
  if (pipe0.fd_read >= 0){
    FD_SET(pipe0.fd_read, &fds_master);
    if (pipe0.fd_read > fdmax) {
      fdmax = pipe0.fd_read;
    }
  }
#endif
#ifdef USE_PIPE1_READER
  if(pipe1.fd_read >= 0){
    FD_SET(pipe1.fd_read, &socks_master);
    if (pipe1.fd_read > fdmax) {
      fdmax = pipe1.fd_read;
    }
  }
#endif
#ifdef USE_PIPE2_READER
  if(pipe2.fd_read >= 0){
    FD_SET(pipe2.fd_read, &socks_master);
    if (pipe2.fd_read > fdmax) {
      fdmax = pipe2.fd_read;
    }
  }
#endif

  /* files to be read, modified after each select */
  fd_set fds;

  while (1) {
    /* reset list of socks to check */
    fds = fds_master;

    if (select(fdmax + 1, &fds, NULL, NULL, NULL) < 0) {
      fprintf(stderr, "pipe_thread: select failed!");
    } else {
#ifdef USE_PIPE0_READER
      if (FD_ISSET(pipe0.fd_read, &fds)) {
        pipe_receive(&pipe0);
      }
#endif
#ifdef USE_PIPE1_READER
       if (FD_ISSET(pipe1.fd_read, &fds)) {
        pipe_receive(&pipe1);
      }
#endif
#ifdef USE_PIPE2_READER
       if (FD_ISSET(pipe2.fd_read, &fds)) {
        pipe_receive(&pipe2);
      }
#endif
    }
  }
  return 0;
}
