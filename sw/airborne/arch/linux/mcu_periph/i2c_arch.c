/*
 *
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2018 Kirk Scheper <kirkscheper@gmail.com>
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

/** @file arch/linux/mcu_periph/i2c_arch.c
 * I2C functionality
 */

#include "mcu_periph/i2c.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <errno.h>

#include <pthread.h>
#include "rt_priority.h"

#ifndef I2C_THREAD_PRIO
#define I2C_THREAD_PRIO 10
#endif

static void *i2c_thread(void *thread_data);

// private I2C init structure
struct i2c_thread_t {
  pthread_mutex_t mutex;
  pthread_cond_t condition;
};

static void UNUSED i2c_arch_init(struct i2c_periph *p)
{
  pthread_t tid;
  if (pthread_create(&tid, NULL, i2c_thread, (void *)p) != 0) {
    fprintf(stderr, "i2c_arch_init: Could not create I2C thread.\n");
    return;
  }
#ifndef __APPLE__
  pthread_setname_np(tid, "i2c");
#endif
}

void i2c_event(void)
{
}

void i2c_setbitrate(struct i2c_periph *p  __attribute__((unused)), int bitrate __attribute__((unused)))
{
}

bool i2c_idle(struct i2c_periph *p __attribute__((unused)))
{
  return true;
}

bool i2c_submit(struct i2c_periph *p, struct i2c_transaction *t)
{
  // get mutex lock and condition
  pthread_mutex_t *mutex = &(((struct i2c_thread_t *)(p->init_struct))->mutex);
  pthread_cond_t *condition = &(((struct i2c_thread_t *)(p->init_struct))->condition);

  uint8_t next_idx;
  pthread_mutex_lock(mutex);
  next_idx = (p->trans_insert_idx + 1) % I2C_TRANSACTION_QUEUE_LEN;
  if (next_idx == p->trans_extract_idx) {
    // queue full
    p->errors->queue_full_cnt++;
    t->status = I2CTransFailed;
    pthread_mutex_unlock(mutex);
    return false;
  }

  t->status = I2CTransPending;

  /* put transaction in queue */
  p->trans[p->trans_insert_idx] = t;
  p->trans_insert_idx = next_idx;

  /* wake handler thread */
  pthread_cond_signal(condition);
  pthread_mutex_unlock(mutex);

  return true;
}

/*
 * Transactions handler thread
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
static void *i2c_thread(void *data)
{
  struct i2c_msg trx_msgs[2];
  struct i2c_rdwr_ioctl_data trx_data = {
    .msgs = trx_msgs,
    .nmsgs = 2
  };

  get_rt_prio(I2C_THREAD_PRIO);

  struct i2c_periph *p = (struct i2c_periph *)data;
  pthread_mutex_t *mutex = &(((struct i2c_thread_t *)(p->init_struct))->mutex);
  pthread_cond_t *condition = &(((struct i2c_thread_t *)(p->init_struct))->condition);

  while (1) {
    /* wait for data to transfer */
    pthread_mutex_lock(mutex);
    if (p->trans_insert_idx == p->trans_extract_idx) {
      pthread_cond_wait(condition, mutex);
    }

    int fd = (int)p->reg_addr;
    struct i2c_transaction *t = p->trans[p->trans_extract_idx];
    pthread_mutex_unlock(mutex);

    // Switch the different transaction types
    switch (t->type) {
      // Just transmitting
      case I2CTransTx:
        // Set the slave address, converted to 7 bit
        ioctl(fd, I2C_SLAVE, t->slave_addr >> 1);
        if (write(fd, (uint8_t *)t->buf, t->len_w) < 0) {
          /* if write failed, increment error counter ack_fail_cnt */
          pthread_mutex_lock(mutex);
          p->errors->ack_fail_cnt++;
          pthread_mutex_unlock(mutex);
          t->status = I2CTransFailed;
        } else {
          t->status = I2CTransSuccess;
        }
        break;
      // Just reading
      case I2CTransRx:
        // Set the slave address, converted to 7 bit
        ioctl(fd, I2C_SLAVE, t->slave_addr >> 1);
        if (read(fd, (uint8_t *)t->buf, t->len_r) < 0) {
          /* if read failed, increment error counter arb_lost_cnt */
          pthread_mutex_lock(mutex);
          p->errors->arb_lost_cnt++;
          pthread_mutex_unlock(mutex);
          t->status = I2CTransFailed;
        } else {
          t->status = I2CTransSuccess;
        }
        break;
      // First Transmit and then read with repeated start
      case I2CTransTxRx:
        trx_msgs[0].addr = t->slave_addr >> 1;
        trx_msgs[0].flags = 0; /* tx */
        trx_msgs[0].len = t->len_w;
        trx_msgs[0].buf = (void *) t->buf;
        trx_msgs[1].addr = t->slave_addr >> 1;
        trx_msgs[1].flags = I2C_M_RD;
        trx_msgs[1].len = t->len_r;
        trx_msgs[1].buf = (void *) t->buf;
        if (ioctl(fd, I2C_RDWR, &trx_data) < 0) {
          /* if write/read failed, increment error counter miss_start_stop_cnt */
          pthread_mutex_lock(mutex);
          p->errors->miss_start_stop_cnt++;
          pthread_mutex_unlock(mutex);
          t->status = I2CTransFailed;
        } else {
          t->status = I2CTransSuccess;
        }
        break;
      default:
        t->status = I2CTransFailed;
        break;
    }

    pthread_mutex_lock(mutex);
    p->trans_extract_idx = (p->trans_extract_idx + 1) % I2C_TRANSACTION_QUEUE_LEN;
    pthread_mutex_unlock(mutex);
  }
  return NULL;
}
#pragma GCC diagnostic pop

#if USE_I2C0
struct i2c_errors i2c0_errors;
struct i2c_thread_t i2c0_thread;

void i2c0_hw_init(void)
{
  i2c0.reg_addr = (void *)open("/dev/i2c-0", O_RDWR);
  i2c0.errors = &i2c0_errors;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c0_errors);

  pthread_mutex_init(&i2c0_thread.mutex, NULL);
  pthread_cond_init(&i2c0_thread.condition, NULL);
  i2c0.init_struct = (void *)(&i2c0_thread);

  i2c_arch_init(&i2c0);
}
#endif

#if USE_I2C1
struct i2c_errors i2c1_errors;
struct i2c_thread_t i2c1_thread;

void i2c1_hw_init(void)
{
  i2c1.reg_addr = (void *)open("/dev/i2c-1", O_RDWR);
  i2c1.errors = &i2c1_errors;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c1_errors);

  pthread_mutex_init(&i2c1_thread.mutex, NULL);
  pthread_cond_init(&i2c1_thread.condition, NULL);
  i2c1.init_struct = (void *)(&i2c1_thread);

  i2c_arch_init(&i2c1);
}
#endif

#if USE_I2C2
struct i2c_errors i2c2_errors;
struct i2c_thread_t i2c2_thread;

void i2c2_hw_init(void)
{
  i2c2.reg_addr = (void *)open("/dev/i2c-2", O_RDWR);
  i2c2.errors = &i2c2_errors;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c2_errors);

  pthread_mutex_init(&i2c2_thread.mutex, NULL);
  pthread_cond_init(&i2c2_thread.condition, NULL);
  i2c2.init_struct = (void *)(&i2c2_thread);

  i2c_arch_init(&i2c2);
}
#endif

#if USE_I2C3
struct i2c_errors i2c3_errors;
struct i2c_thread_t i2c3_thread;

void i2c3_hw_init(void)
{
  i2c3.reg_addr = (void *)open("/dev/i2c-3", O_RDWR);
  i2c3.errors = &i2c3_errors;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c3_errors);

  pthread_mutex_init(&i2c3_thread.mutex, NULL);
  pthread_cond_init(&i2c3_thread.condition, NULL);
  i2c3.init_struct = (void *)(&i2c3_thread);

  i2c_arch_init(&i2c3);
}
#endif
