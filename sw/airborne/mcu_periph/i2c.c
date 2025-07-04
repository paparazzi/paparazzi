/*
 * Copyright (C) 2010-2012 The Paparazzi Team
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
 * @file mcu_periph/i2c.c
 * Architecture independent I2C (Inter-Integrated Circuit Bus) API.
 */

#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"

#define USE_I2C (USE_I2C0 || USE_I2C1 || USE_I2C2 || USE_I2C3 || USE_I2C4)
#define USE_SOFT_I2C (USE_SOFTI2C0 || USE_SOFTI2C1)

#if PERIODIC_TELEMETRY && (USE_I2C || USE_SOFT_I2C)
#include "modules/datalink/telemetry.h"

static void send_i2cx_err(struct transport_tx *trans, struct link_device *dev, struct i2c_periph* i2c)
{
  uint16_t i2c_wd_reset_cnt          = i2c->errors->wd_reset_cnt;
  uint16_t i2c_queue_full_cnt        = i2c->errors->queue_full_cnt;
  uint16_t i2c_ack_fail_cnt          = i2c->errors->ack_fail_cnt;
  uint16_t i2c_miss_start_stop_cnt   = i2c->errors->miss_start_stop_cnt;
  uint16_t i2c_arb_lost_cnt          = i2c->errors->arb_lost_cnt;
  uint16_t i2c_over_under_cnt        = i2c->errors->over_under_cnt;
  uint16_t i2c_pec_recep_cnt         = i2c->errors->pec_recep_cnt;
  uint16_t i2c_timeout_tlow_cnt      = i2c->errors->timeout_tlow_cnt;
  uint16_t i2c_smbus_alert_cnt       = i2c->errors->smbus_alert_cnt;
  uint16_t i2c_unexpected_event_cnt  = i2c->errors->unexpected_event_cnt;
  uint32_t i2c_last_unexpected_event = i2c->errors->last_unexpected_event;
  uint8_t _bus = 0;
  pprz_msg_send_I2C_ERRORS(trans, dev, AC_ID,
                           &i2c_wd_reset_cnt,
                           &i2c_queue_full_cnt,
                           &i2c_ack_fail_cnt,
                           &i2c_miss_start_stop_cnt,
                           &i2c_arb_lost_cnt,
                           &i2c_over_under_cnt,
                           &i2c_pec_recep_cnt,
                           &i2c_timeout_tlow_cnt,
                           &i2c_smbus_alert_cnt,
                           &i2c_unexpected_event_cnt,
                           &i2c_last_unexpected_event,
                           &_bus);
}
#endif


#if USE_I2C0

struct i2c_periph i2c0;

void i2c0_init(void)
{
  i2c_init(&i2c0);
  i2c0_hw_init();
}

#endif /* USE_I2C0 */


#if USE_I2C1

struct i2c_periph i2c1;

void i2c1_init(void)
{
  i2c_init(&i2c1);
  i2c1_hw_init();
}

#endif /* USE_I2C1 */


#if USE_I2C2

struct i2c_periph i2c2;

void i2c2_init(void)
{
  i2c_init(&i2c2);
  i2c2_hw_init();
}

#endif /* USE_I2C2 */

#if USE_I2C3

struct i2c_periph i2c3;

void i2c3_init(void)
{
  i2c_init(&i2c3);
  i2c3_hw_init();
}

#endif /* USE_I2C3 */

#if USE_I2C4

struct i2c_periph i2c4;

void i2c4_init(void)
{
  i2c_init(&i2c4);
  i2c4_hw_init();
}

#endif /* USE_I2C4 */

#if USE_SOFTI2C0
extern void send_softi2c0_err(struct transport_tx *trans, struct link_device *dev);
#endif /* USE_SOFTI2C0 */

#if USE_SOFTI2C1
extern void send_softi2c1_err(struct transport_tx *trans, struct link_device *dev);
#endif /* USE_SOFTI2C1 */

#if PERIODIC_TELEMETRY && (USE_I2C || USE_SOFT_I2C)
static void send_i2c_err(struct transport_tx *trans __attribute__((unused)),
                         struct link_device *dev __attribute__((unused)))
{
  static uint8_t _i2c_nb_cnt = 0;
  switch (_i2c_nb_cnt) {
    case 0:
#if USE_I2C0
      send_i2cx_err(trans, dev, &i2c0);
#endif
      break;
    case 1:
#if USE_I2C1
      send_i2cx_err(trans, dev, &i2c1);
#endif
      break;
    case 2:
#if USE_I2C2
      send_i2cx_err(trans, dev, &i2c2);
#endif
      break;
    case 3:
#if USE_I2C3
      send_i2cx_err(trans, dev, &i2c3);
#endif
      break;
    case 4:
#if USE_I2C4
      send_i2cx_err(trans, dev, &i2c4);
#endif
    case 5:
#if USE_SOFTI2C0
      send_softi2c0_err(trans, dev);
#endif
    case 6:
#if USE_SOFTI2C1
      send_softi2c1_err(trans, dev);
#endif
      break;
    default:
      break;
  }
  _i2c_nb_cnt++;
  if (_i2c_nb_cnt == 7) {
    _i2c_nb_cnt = 0;
  }
}
#endif


void i2c_init(struct i2c_periph *p)
{
  p->trans_insert_idx = 0;
  p->trans_extract_idx = 0;
  p->status = I2CIdle;
  p->reg_addr = NULL;

#if PERIODIC_TELEMETRY && (USE_I2C || USE_SOFT_I2C)
  // the first to register do it for the others
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_I2C_ERRORS, send_i2c_err);
#endif
}


bool i2c_transmit(struct i2c_periph *p, struct i2c_transaction *t,
                  uint8_t s_addr, uint8_t len)
{
  t->type = I2CTransTx;
  t->slave_addr = s_addr;
  t->len_w = len;
  t->len_r = 0;
  return i2c_submit(p, t);
}

bool i2c_receive(struct i2c_periph *p, struct i2c_transaction *t,
                 uint8_t s_addr, uint16_t len)
{
  t->type = I2CTransRx;
  t->slave_addr = s_addr;
  t->len_w = 0;
  t->len_r = len;
  return i2c_submit(p, t);
}

bool i2c_transceive(struct i2c_periph *p, struct i2c_transaction *t,
                    uint8_t s_addr, uint8_t len_w, uint16_t len_r)
{
  t->type = I2CTransTxRx;
  t->slave_addr = s_addr;
  t->len_w = len_w;
  t->len_r = len_r;
  return i2c_submit(p, t);
}

static enum I2CTransactionStatus i2c_blocking_submit(struct i2c_periph *p, struct i2c_transaction *t, float timeout) {
  if (!i2c_submit(p, t)) {
    return I2CTransFailed;
  }

  // Wait for transaction to complete
  if(pprz_bsem_wait_timeout(&t->bsem, timeout) == 0) {
    return t->status;
  } else {
    return I2CTransFailed;
  }
}

enum I2CTransactionStatus i2c_blocking_transmit(struct i2c_periph *p, struct i2c_transaction *t,
                           uint8_t s_addr, uint8_t len, float timeout)
{
  t->type = I2CTransTx;
  t->slave_addr = s_addr;
  t->len_w = len;
  t->len_r = 0;
  return i2c_blocking_submit(p, t, timeout);
}

enum I2CTransactionStatus i2c_blocking_receive(struct i2c_periph *p, struct i2c_transaction *t,
                          uint8_t s_addr, uint16_t len, float timeout)
{
  t->type = I2CTransRx;
  t->slave_addr = s_addr;
  t->len_w = 0;
  t->len_r = len;
  return i2c_blocking_submit(p, t, timeout);
}

enum I2CTransactionStatus i2c_blocking_transceive(struct i2c_periph *p, struct i2c_transaction *t,
                             uint8_t s_addr, uint8_t len_w, uint16_t len_r, float timeout)
{
  t->type = I2CTransTxRx;
  t->slave_addr = s_addr;
  t->len_w = len_w;
  t->len_r = len_r;
  return i2c_blocking_submit(p, t, timeout);
}
