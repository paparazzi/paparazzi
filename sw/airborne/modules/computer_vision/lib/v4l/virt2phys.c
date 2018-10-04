#define _LARGEFILE64_SOURCE     /* See feature_test_macros(7) */

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "virt2phys.h"

/* How it works
 *  Source : See the file Documentation/wm/pagemap.txt in linux Documentation
 *
 * The file /proc/pid/pagemap contains 8 bytes for each virtual PFN describing
 * the page status. First we check the bits indicating if page is present and
 * not swapped. Then we can get the physical PFN.
 *
 * Contiguity check consist to verify of contiguous virtual memory correspond
 * to present, not swapped and contiguous physical pages.
 */

#define BIT(i)          ((unsigned long long) 1 << i)
#define FILENAME_SIZE   30
#define PAGE_SHIFT      12
#define PAGE_SIZE       (1<<PAGE_SHIFT)
#define PAGE_MASK       (((unsigned long long)1<< PAGE_SHIFT) -1)

#define PAGE_PRESENT    BIT(63)
#define PAGE_SWAPPED    BIT(62)
#define PAGE_PFN_MASK   (((unsigned long long)1<< 55) -1)

static int open_pagemap(pid_t pid)
{
	char filename[FILENAME_SIZE];

	if ( 0 > snprintf(filename,FILENAME_SIZE,"/proc/%d/pagemap",pid)) {
		fprintf(stderr,"Cant create file name\n");
		return -1;
	}

	return open(filename, O_RDONLY);
}

static int virt2phys(int fd, unsigned long vaddr, unsigned long *paddr)
{
	int                     rt = -1;
	unsigned long long      pm_info, index;

	index = (vaddr >> PAGE_SHIFT) * sizeof(unsigned long long);

	if ((off64_t)index != lseek64(fd,index,SEEK_SET))
		return rt;

	/* This read can return 0 if address is not in userspace */
	if (sizeof(unsigned long long) != read(fd, &pm_info, sizeof(unsigned long long)))
		return rt;

	if (((pm_info & PAGE_PRESENT) && (!(pm_info & PAGE_SWAPPED)))) {
		*paddr = ((pm_info & PAGE_PFN_MASK) << PAGE_SHIFT) \
		         + (vaddr & PAGE_MASK);
		rt = 0;
	}
	return rt;
}

int checkcontiguity(unsigned long vaddr,
                    pid_t pid,
                    struct physmem * pmem,
                    size_t size)
{
	int fd;
	unsigned long vcurrent, pcurrent, pnext;

	pmem->paddr = 0;
	pmem->size  = 0;

	fd = open_pagemap(pid);
	if (fd < 0)
		return -1;

	/* Get first physical address, it may not be aligned on a page */
	if(virt2phys(fd,vaddr,&pmem->paddr)) {
		close(fd);
		return -1;
	}

	pmem->size = PAGE_SIZE - (pmem->paddr & PAGE_MASK);
	/* We aligned addresses on pages */
	vcurrent = vaddr & ~PAGE_MASK;
	pnext = (pmem->paddr & ~PAGE_MASK) + PAGE_SIZE;

	while (1) {
		/* exit if we had checked the requested bytes */
		if ((size) && (pmem->size >= size))
			break;

		vcurrent += PAGE_SIZE;
		if (virt2phys(fd, vcurrent, &pcurrent))
			break ;

		/* Test if memory is still contiguous */
		if (pcurrent == pnext) {
			pmem->size += PAGE_SIZE;
			pnext+= PAGE_SIZE;
		} else {
			break;
		}
	}
	close(fd);
	return 0;
}
