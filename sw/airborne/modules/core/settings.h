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
 * @file modules/core/settings.h
 * Persistent settings interface.
 *
 */

#ifndef CORE_SETTINGS_H
#define CORE_SETTINGS_H

#include "std.h"

extern void settings_init(void);
extern int32_t settings_store(void);
extern int32_t settings_clear(void);

extern bool settings_store_flag;
extern bool settings_clear_flag;

#define settings_StoreSettings(_v) { settings_store_flag = _v; settings_store(); }
#define settings_ClearSettings(_v) { settings_clear_flag = _v; settings_clear(); }

/* implemented in arch dependant code */
int32_t persistent_write(void *ptr, uint32_t size);
int32_t persistent_read(void *ptr, uint32_t size);
int32_t persistent_clear(void);


#endif /* CORE_SETTINGS_H */
