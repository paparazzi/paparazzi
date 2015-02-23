/*
 * Copyright (C) 2009-2015 The Paparazzi Team
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
 * @file arch/linux/subsystems/settings_arch.c
 * linux arch Persistent settings.
 *
 * Saves the PersistentSettings struct to a binary file.
 */

#include "subsystems/settings.h"
#include <stdio.h>

/** Default file used to store persistent settings */
#ifndef PERSISTENT_SETTINGS_FILE
#define PERSISTENT_SETTINGS_FILE "pprz_persistent_settings.binary"
#endif

int32_t persistent_write(void *ptr, uint32_t size)
{
  FILE *file= fopen(PERSISTENT_SETTINGS_FILE, "wb");
  if (file != NULL) {
    fwrite(ptr, size, 1, file);
    fclose(file);
    return 0;
  }
  printf("Could not open settings file %s to write!\n", PERSISTENT_SETTINGS_FILE);
  return -1;
}

int32_t persistent_read(void *ptr, uint32_t size)
{
  FILE *file= fopen(PERSISTENT_SETTINGS_FILE, "rb");
  if (file == NULL) {
    printf("Could not open settings file %s to read!\n", PERSISTENT_SETTINGS_FILE);
    return -1;
  }
  /* check if binary file size matches requested struct size */
  fseek(file, 0, SEEK_END);
  if (ftell(file) != size) {
    printf("Settings file %s size does not match, deleting it!\n", PERSISTENT_SETTINGS_FILE);
    fclose(file);
    remove(PERSISTENT_SETTINGS_FILE);
    return -1;
  }
  fseek(file, 0, SEEK_SET);
  int bytes_read = fread(ptr, size, 1, file);
  if (bytes_read != size) {
    printf("Could only read %d of %d bytes from %s!\n", bytes_read, size,
           PERSISTENT_SETTINGS_FILE);
  }
  fclose(file);
  return 0;
}
