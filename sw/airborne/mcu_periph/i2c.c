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

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

#if USE_I2C0

struct i2c_periph i2c0;

#if PERIODIC_TELEMETRY
static void send_i2c0_err(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t i2c0_wd_reset_cnt          = i2c0.errors->wd_reset_cnt;
  uint16_t i2c0_queue_full_cnt        = i2c0.errors->queue_full_cnt;
  uint16_t i2c0_ack_fail_cnt          = i2c0.errors->ack_fail_cnt;
  uint16_t i2c0_miss_start_stop_cnt   = i2c0.errors->miss_start_stop_cnt;
  uint16_t i2c0_arb_lost_cnt          = i2c0.errors->arb_lost_cnt;
  uint16_t i2c0_over_under_cnt        = i2c0.errors->over_under_cnt;
  uint16_t i2c0_pec_recep_cnt         = i2c0.errors->pec_recep_cnt;
  uint16_t i2c0_timeout_tlow_cnt      = i2c0.errors->timeout_tlow_cnt;
  uint16_t i2c0_smbus_alert_cnt       = i2c0.errors->smbus_alert_cnt;
  uint16_t i2c0_unexpected_event_cnt  = i2c0.errors->unexpected_event_cnt;
  uint32_t i2c0_last_unexpected_event = i2c0.errors->last_unexpected_event;
  uint8_t _bus0 = 0;
  pprz_msg_send_I2C_ERRORS(trans, dev, AC_ID,
                           &i2c0_wd_reset_cnt,
                           &i2c0_queue_full_cnt,
                           &i2c0_ack_fail_cnt,
                           &i2c0_miss_start_stop_cnt,
                           &i2c0_arb_lost_cnt,
                           &i2c0_over_under_cnt,
                           &i2c0_pec_recep_cnt,
                           &i2c0_timeout_tlow_cnt,
                           &i2c0_smbus_alert_cnt,
                           &i2c0_unexpected_event_cnt,
                           &i2c0_last_unexpected_event,
                           &_bus0);
}
#endif

void i2c0_init(void)
{
  i2c_init(&i2c0);
  i2c0_hw_init();
}

#endif /* USE_I2C0 */


#if USE_I2C1

struct i2c_periph i2c1;

#if PERIODIC_TELEMETRY
static void send_i2c1_err(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t i2c1_wd_reset_cnt          = i2c1.errors->wd_reset_cnt;
  uint16_t i2c1_queue_full_cnt        = i2c1.errors->queue_full_cnt;
  uint16_t i2c1_ack_fail_cnt          = i2c1.errors->ack_fail_cnt;
  uint16_t i2c1_miss_start_stop_cnt   = i2c1.errors->miss_start_stop_cnt;
  uint16_t i2c1_arb_lost_cnt          = i2c1.errors->arb_lost_cnt;
  uint16_t i2c1_over_under_cnt        = i2c1.errors->over_under_cnt;
  uint16_t i2c1_pec_recep_cnt         = i2c1.errors->pec_recep_cnt;
  uint16_t i2c1_timeout_tlow_cnt      = i2c1.errors->timeout_tlow_cnt;
  uint16_t i2c1_smbus_alert_cnt       = i2c1.errors->smbus_alert_cnt;
  uint16_t i2c1_unexpected_event_cnt  = i2c1.errors->unexpected_event_cnt;
  uint32_t i2c1_last_unexpected_event = i2c1.errors->last_unexpected_event;
  uint8_t _bus1 = 1;
  pprz_msg_send_I2C_ERRORS(trans, dev, AC_ID,
                           &i2c1_wd_reset_cnt,
                           &i2c1_queue_full_cnt,
                           &i2c1_ack_fail_cnt,
                           &i2c1_miss_start_stop_cnt,
                           &i2c1_arb_lost_cnt,
                           &i2c1_over_under_cnt,
                           &i2c1_pec_recep_cnt,
                           &i2c1_timeout_tlow_cnt,
                           &i2c1_smbus_alert_cnt,
                           &i2c1_unexpected_event_cnt,
                           &i2c1_last_unexpected_event,
                           &_bus1);
}
#endif

void i2c1_init(void)
{
  i2c_init(&i2c1);
  i2c1_hw_init();
}

#endif /* USE_I2C1 */


#if USE_I2C2

struct i2c_periph i2c2;

#if PERIODIC_TELEMETRY
static void send_i2c2_err(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t i2c2_wd_reset_cnt          = i2c2.errors->wd_reset_cnt;
  uint16_t i2c2_queue_full_cnt        = i2c2.errors->queue_full_cnt;
  uint16_t i2c2_ack_fail_cnt          = i2c2.errors->ack_fail_cnt;
  uint16_t i2c2_miss_start_stop_cnt   = i2c2.errors->miss_start_stop_cnt;
  uint16_t i2c2_arb_lost_cnt          = i2c2.errors->arb_lost_cnt;
  uint16_t i2c2_over_under_cnt        = i2c2.errors->over_under_cnt;
  uint16_t i2c2_pec_recep_cnt         = i2c2.errors->pec_recep_cnt;
  uint16_t i2c2_timeout_tlow_cnt      = i2c2.errors->timeout_tlow_cnt;
  uint16_t i2c2_smbus_alert_cnt       = i2c2.errors->smbus_alert_cnt;
  uint16_t i2c2_unexpected_event_cnt  = i2c2.errors->unexpected_event_cnt;
  uint32_t i2c2_last_unexpected_event = i2c2.errors->last_unexpected_event;
  uint8_t _bus2 = 2;
  pprz_msg_send_I2C_ERRORS(trans, dev, AC_ID,
                           &i2c2_wd_reset_cnt,
                           &i2c2_queue_full_cnt,
                           &i2c2_ack_fail_cnt,
                           &i2c2_miss_start_stop_cnt,
                           &i2c2_arb_lost_cnt,
                           &i2c2_over_under_cnt,
                           &i2c2_pec_recep_cnt,
                           &i2c2_timeout_tlow_cnt,
                           &i2c2_smbus_alert_cnt,
                           &i2c2_unexpected_event_cnt,
                           &i2c2_last_unexpected_event,
                           &_bus2);
}
#endif

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

#if PERIODIC_TELEMETRY
static void send_i2c3_err(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t i2c3_wd_reset_cnt          = i2c3.errors->wd_reset_cnt;
  uint16_t i2c3_queue_full_cnt        = i2c3.errors->queue_full_cnt;
  uint16_t i2c3_ack_fail_cnt          = i2c3.errors->ack_fail_cnt;
  uint16_t i2c3_miss_start_stop_cnt   = i2c3.errors->miss_start_stop_cnt;
  uint16_t i2c3_arb_lost_cnt          = i2c3.errors->arb_lost_cnt;
  uint16_t i2c3_over_under_cnt        = i2c3.errors->over_under_cnt;
  uint16_t i2c3_pec_recep_cnt         = i2c3.errors->pec_recep_cnt;
  uint16_t i2c3_timeout_tlow_cnt      = i2c3.errors->timeout_tlow_cnt;
  uint16_t i2c3_smbus_alert_cnt       = i2c3.errors->smbus_alert_cnt;
  uint16_t i2c3_unexpected_event_cnt  = i2c3.errors->unexpected_event_cnt;
  uint32_t i2c3_last_unexpected_event = i2c3.errors->last_unexpected_event;
  uint8_t _bus3 = 3;
  pprz_msg_send_I2C_ERRORS(trans, dev, AC_ID,
                           &i2c3_wd_reset_cnt,
                           &i2c3_queue_full_cnt,
                           &i2c3_ack_fail_cnt,
                           &i2c3_miss_start_stop_cnt,
                           &i2c3_arb_lost_cnt,
                           &i2c3_over_under_cnt,
                           &i2c3_pec_recep_cnt,
                           &i2c3_timeout_tlow_cnt,
                           &i2c3_smbus_alert_cnt,
                           &i2c3_unexpected_event_cnt,
                           &i2c3_last_unexpected_event,
                           &_bus3);
}
#endif

#endif /* USE_I2C3 */

#if PERIODIC_TELEMETRY
static void send_i2c_err(struct transport_tx *trans __attribute__((unused)),
                         struct link_device *dev __attribute__((unused)))
{
  static uint8_t _i2c_nb_cnt = 0;
  switch (_i2c_nb_cnt) {
    case 0:
#if USE_I2C0
      send_i2c0_err(trans, dev);
#endif
      break;
    case 1:
#if USE_I2C1
      send_i2c1_err(trans, dev);
#endif
      break;
    case 2:
#if USE_I2C2
      send_i2c2_err(trans, dev);
#endif
      break;
    case 3:
#if USE_I2C3
      send_i2c3_err(trans, dev);
#endif
      break;
    default:
      break;
  }
  _i2c_nb_cnt++;
  if (_i2c_nb_cnt == 4) {
    _i2c_nb_cnt = 0;
  }
}
#endif


void i2c_init(struct i2c_periph *p)
{
  p->trans_insert_idx = 0;
  p->trans_extract_idx = 0;
  p->status = I2CIdle;

#if PERIODIC_TELEMETRY
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
