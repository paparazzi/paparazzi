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
