/*
 * Copyright (C) 2019 Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/lib/v4l/virt2phys.c
 * Mapping of virtual to physical memory
 *
 *  Source : See the linux kernel documentation at -> https://www.kernel.org/doc/Documentation/vm/pagemap.txt
 *
 * The file /proc/pid/pagemap contains 8 bytes for each virtual PFN describing
 * the page status. First we check the bits indicating if page is present and
 * not swapped. Then we can get the physical PFN.
 *
 * Contiguity check consist to verify of contiguous virtual memory correspond
 * to present, not swapped and contiguous physical pages.
 */

/* Will make the 64 bit interfaces available (off64_t, lseek64(), etc...). See feature_test_macros(7) */
#define _LARGEFILE64_SOURCE

#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "virt2phys.h"

#define BIT(i)          ((unsigned long long) 1 << i)
#define FILENAME_SIZE   30
#define PAGE_SHIFT      12
#define PAGE_SIZE       (1<<PAGE_SHIFT)
#define PAGE_MASK       (((unsigned long long)1<< PAGE_SHIFT) -1)

#define PAGE_PRESENT    BIT(63)
#define PAGE_SWAPPED    BIT(62)
#define PAGE_PFN_MASK   (((unsigned long long)1<< 55) -1)

/*
 * open_pagemap - open pagemap
 * @param[in] pid: process id of program with pagemap to access
 * @return file descriptor of pagemap, -1 if unable to open
 */
static int open_pagemap(pid_t pid)
{
  char filename[FILENAME_SIZE];

  if (snprintf(filename, FILENAME_SIZE, "/proc/%d/pagemap", pid) < 0) {
    fprintf(stderr, "Cant create file name\n");
    return -1;
  }

  return open(filename, O_RDONLY);
}

/*
 * virt2phys - Get physical address of virtual memory
 * @param[in] fd    : file descriptor of pagemap
 * @param[in] vaddr : virtual address
 * @param[in] *paddr: physical address
 * @return 0 if successful, -1 otherwise
 */
static int virt2phys(int fd, unsigned long vaddr, unsigned long *paddr)
{
  unsigned long long pm_info, index;

  index = (vaddr >> PAGE_SHIFT) * sizeof(unsigned long long);

  if ((off64_t)index != lseek64(fd, index, SEEK_SET)) {
    return -1;
  }

  /* This read can return 0 if address is not in userspace */
  if (sizeof(unsigned long long) != read(fd, &pm_info, sizeof(unsigned long long))) {
    return -1;
  }

  if ((pm_info & PAGE_PRESENT) && (!(pm_info & PAGE_SWAPPED))) {
    *paddr = ((pm_info & PAGE_PFN_MASK) << PAGE_SHIFT) + (vaddr & PAGE_MASK);
  }

  return 0;
}

/* checkcontiguity - Get information on physical memory associated with virtual address.
 *
 * @param[in] vaddr : the virtual address in the context of process pid
 * @param[in] pid   : a valid pid
 * @param[in] *pmem : Valid only if fonction returns 0.
 *            pmem->paddr contains physical address of the first byte.
 *            pmem->size contains the number of contiguous bytes checked. If size is not zero, buffer is contiguous if pmem->size >= size.
 * @param[in] size  : if size>0 contiguity is checked as far as possible.
 *                  : if size=0, check contiguity until at least size bytes had been checked as contiguous.
 * @return 0 if vaddr can be converted in physical address and -1 otherwise.
 */
int check_contiguity(unsigned long vaddr, pid_t pid, struct physmem *pmem, size_t size)
{
  int fd;
  unsigned long vcurrent, pcurrent = 0, pnext;

  pmem->paddr = 0;
  pmem->size  = 0;

  fd = open_pagemap(pid);
  if (fd < 0) {
    return -1;
  }

  /* Get first physical address, it may not be aligned on a page */
  if (virt2phys(fd, vaddr, &pmem->paddr)) {
    close(fd);
    return -1;
  }

  pmem->size = PAGE_SIZE - (pmem->paddr & PAGE_MASK);
  /* We aligned addresses on pages */
  vcurrent = vaddr & ~PAGE_MASK;
  pnext = (pmem->paddr & ~PAGE_MASK) + PAGE_SIZE;

  while (1) {
    /* exit if we had checked the requested bytes */
    if (size && (pmem->size >= size)) {
      break;
    }

    vcurrent += PAGE_SIZE;
    if (virt2phys(fd, vcurrent, &pcurrent)) {
      break;
    }

    /* Test if memory is still contiguous */
    if (pcurrent == pnext) {
      pmem->size += PAGE_SIZE;
      pnext += PAGE_SIZE;
    } else {
      break;
    }
  }
  close(fd);

  return 0;
}
