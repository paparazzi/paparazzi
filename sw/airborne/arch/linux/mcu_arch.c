/*
 *
 * Copyright (C) 2009-2013 The Paparazzi Team
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
 * @file arch/linux/mcu_arch.h
 * linux arch dependant microcontroller initialisation functions.
 *
 * Because Linux runs on omap, we don't have to initialize the MCU ourselves.
 */

#include "mcu_arch.h"
#include "mcu.h"

#include <stdio.h>
#include <string.h>
#include <dirent.h>

/** Optional file with a unique board serial number (read as text) */
#ifndef LINUX_UID_FILE
#define LINUX_UID_FILE ""
#endif

/**
 * Read the first line of a text file as unique ID
 */
static uint8_t uid_from_file(const char *path, uint8_t *uid, uint8_t max_len)
{
  char line[64];
  if (path[0] == '\0') {
    return 0;
  }
  FILE *f = fopen(path, "r");
  if (f == NULL) {
    return 0;
  }
  uint8_t len = 0;
  if (fgets(line, sizeof(line), f) != NULL) {
    line[strcspn(line, "\r\n")] = '\0';
    len = Min(strlen(line), max_len);
    memcpy(uid, line, len);
  }
  fclose(f);
  return len;
}

/**
 * Get the MAC address of the first network interface (by name), skipping loopback
 */
static uint8_t uid_from_mac(uint8_t *uid, uint8_t max_len)
{
  DIR *dir = opendir("/sys/class/net");
  if (dir == NULL) {
    return 0;
  }
  char best_name[64] = "";
  uint8_t best_mac[6];
  struct dirent *ent;
  while ((ent = readdir(dir)) != NULL) {
    char path[300];
    unsigned int m[6];
    if (ent->d_name[0] == '.' || strcmp(ent->d_name, "lo") == 0 ||
        strlen(ent->d_name) >= sizeof(best_name) ||
        (best_name[0] != '\0' && strcmp(ent->d_name, best_name) >= 0)) {
      continue;
    }
    snprintf(path, sizeof(path), "/sys/class/net/%s/address", ent->d_name);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
      continue;
    }
    int n = fscanf(f, "%x:%x:%x:%x:%x:%x", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]);
    fclose(f);
    if (n != 6 || (m[0] | m[1] | m[2] | m[3] | m[4] | m[5]) == 0) {
      continue;
    }
    strcpy(best_name, ent->d_name);
    for (int i = 0; i < 6; i++) {
      best_mac[i] = m[i];
    }
  }
  closedir(dir);
  if (best_name[0] == '\0') {
    return 0;
  }
  uint8_t len = Min(6, max_len);
  memcpy(uid, best_mac, len);
  return len;
}

/**
 * Get a unique hardware ID of the board.
 * Uses the serial number from LINUX_UID_FILE if set and readable,
 * otherwise the MAC address of the first network interface.
 */
uint8_t mcu_get_uid(uint8_t *uid, uint8_t max_len)
{
  uint8_t len = uid_from_file(LINUX_UID_FILE, uid, max_len);
  if (len == 0) {
    len = uid_from_mac(uid, max_len);
  }
  return len;
}

#if USE_LINUX_SIGNAL
#include "message_pragmas.h"
PRINT_CONFIG_MSG("Catching SIGINT. Press CTRL-C twice to stop program.")

/*
 * Handle linux signals by hand if the program is not launch
 * by a shell already doing it
 */

#include <stdlib.h>
#include <stdio.h>
#include <signal.h>

/**
 * catch SIGINT signal two times to stop the main program
 */
static void sig_handler(int signo)
{
  static int nb_signal = 0;

  if (signo == SIGINT) {
    printf("Received SIGINT\n");
    if (nb_signal == 0) {
      printf("Press Ctrl-C again to stop the program\n");
      nb_signal++;
    } else {
      printf("Leaving now\n");
      exit(0);
    }
  }
}

void mcu_arch_init(void)
{
  struct sigaction sa;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sa.sa_handler = &sig_handler;
  if (sigaction(SIGINT, &sa, NULL) == -1) {
    printf("Can't catch SIGINT\n");
  }
}

#else

void mcu_arch_init(void) { }

#endif

