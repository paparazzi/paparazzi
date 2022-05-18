/*
 * Copyright (C) Alexandre Bustico <alexandre.bustico@enac.fr>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/core/shell_arch.c"
 * @author Alexandre Bustico <alexandre.bustico@enac.fr>
 * Simple debug shell
 */

#include "modules/core/shell.h"
#include "modules/core/microrl/microrlShell.h"
#include "mcu_periph/uart.h"
#include "printf.h"
#include "modules/core/abi.h"

/*************************
 * Basic static commands *
 *************************/

static void cmd_mem(BaseSequentialStream *lchp, int argc, const char *const argv[])
{
  size_t n, total, largest;
  memory_area_t area;

  (void)argv;
  if (argc > 0) {
    chprintf(lchp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &total, &largest);
  chCoreGetStatusX(&area);
  chprintf(lchp, "core free memory : %u bytes\r\n", area.size);
  chprintf(lchp, "heap fragments   : %u\r\n", n);
  chprintf(lchp, "heap free total  : %u bytes\r\n", total);
  chprintf(lchp, "heap free largest: %u bytes\r\n", largest);
}

static void cmd_abi(BaseSequentialStream *lchp, int argc, const char *const argv[])
{
  (void)argv;
  if (argc > 0) {
    chprintf(lchp, "Usage: abi\r\n");
    return;
  }

  chprintf(lchp, "ABI message bindings\r\n");
  for (int i = 0; i < ABI_MESSAGE_NB; i++) {
    chprintf(lchp, " msg %d: ", i);
    if (abi_queues[i] == NULL) {
      chprintf(lchp, "no bindings\r\n");
    } else {
      abi_event *e;
      ABI_FOREACH(abi_queues[i], e) {
        chprintf(lchp, "(cb 0x%lx, id %d), ", e->cb, e->id);
      }
      chprintf(lchp, "\r\n");
    }
  }
}

static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"abi", cmd_abi},
  {NULL, NULL}
};

static ShellConfig shell_cfg = {
  NULL,
  commands
};

/**
 * Add dynamic entry
 */
void shell_add_entry(char *cmd_name, shell_cmd_t *cmd)
{
  shellAddEntry((ShellCommand) {cmd_name, cmd});
}


/** Arch init
 */
void shell_init_arch(void)
{
  // This should be called after mcu periph init
  shell_cfg.sc_channel = (BaseSequentialStream *)(SHELL_DEV.reg_addr);

  shellInit();
  thread_t *shelltp = shellCreateFromHeap(&shell_cfg, 2048U, NORMALPRIO);
  if (shelltp == NULL) {
    chSysHalt("fail starting shell");
  }
}

