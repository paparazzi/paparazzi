/*
 * Copyright (C) 2009-2014 The Paparazzi Team
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
 */

/**
 * @file modules/core/settings.c
 * Persistent settings interface.
 *
 */

#include "modules/core/settings.h"
#include "generated/settings.h"
#include "generated/airframe.h"
#include "pprzlink/messages.h"
#include "pprzlink/dl_protocol.h"

struct PersistentSettings pers_settings;

/** flag for setting feedback.
 * if TRUE, was stored sucessfully.
 * Also settings still need a variable,
 * pure function call not possible yet.
 */
bool settings_store_flag;

bool settings_clear_flag;


void settings_init(void)
{
#if USE_PERSISTENT_SETTINGS
  if (persistent_read((void *)&pers_settings, sizeof(struct PersistentSettings))) {
    return;  // return -1 ?
  }
  /* from generated/settings.h */
  persistent_settings_load();
#endif
}

/** store settings marked as persistent to flash
 * @return 0 on success
 */
int32_t settings_store(void)
{
#if USE_PERSISTENT_SETTINGS
  if (settings_store_flag) {
    /* from generated/settings.h */
    persistent_settings_store();
    if (!persistent_write((void *)&pers_settings, sizeof(struct PersistentSettings))) {
      /* persistent write was successful */
      settings_store_flag = true;
      return 0;
    }
  }
#endif
  settings_store_flag = false;
  return -1;
}

/** clear all persistent settings from flash
 * @return 0 on success
 */
int32_t settings_clear(void)
{
#if USE_PERSISTENT_SETTINGS
  if (settings_clear_flag) {
    if (!persistent_clear()) {
      /* clearing all persistent settings was successful */
      settings_clear_flag = true;
      return 0;
    }
  }
#endif
  settings_clear_flag = false;
  return -1;
}

void settings_parse_msg_SETTING(struct link_device *dev, struct transport_tx *trans, uint8_t *buf)
{
#ifndef INTERMCU_FBW
  if (DL_SETTING_ac_id(buf) != AC_ID) { return; }
  uint8_t sender_id = pprzlink_get_msg_sender_id(buf);
  uint8_t i = DL_SETTING_index(buf);
  float var = DL_SETTING_value(buf);
  DlSetting(i, var);
  // Reply to the sender of the message
  struct pprzlink_msg msg;
  msg.trans = trans;
  msg.dev = dev;
  msg.sender_id = AC_ID;
  msg.receiver_id = sender_id;
  msg.component_id = 0;
  pprzlink_msg_send_DL_VALUE(&msg, &i, &var);
#else
  (void)dev;
  (void)trans;
  (void)buf;
#endif
}

void settings_parse_msg_GET_SETTING(struct link_device *dev, struct transport_tx *trans, uint8_t *buf)
{
#ifndef INTERMCU_FBW
  if (DL_GET_SETTING_ac_id(buf) != AC_ID) { return; }
  uint8_t sender_id = pprzlink_get_msg_sender_id(buf);
  uint8_t i = DL_GET_SETTING_index(buf);
  float val = settings_get_value(i);
  // Reply to the sender of the message
  struct pprzlink_msg msg;
  msg.trans = trans;
  msg.dev = dev;
  msg.sender_id = AC_ID;
  msg.receiver_id = sender_id;
  msg.component_id = 0;
  pprzlink_msg_send_DL_VALUE(&msg, &i, &val);
#else
  (void)dev;
  (void)trans;
  (void)buf;
#endif
}

