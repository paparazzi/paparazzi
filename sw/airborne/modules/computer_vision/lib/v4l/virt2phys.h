#ifndef PARROT_VIRT2PHYS
#define PARROT_VIRT2PHYS

#include <sys/types.h>
#include <unistd.h>

struct physmem {
  unsigned long paddr;
  size_t        size;
};

/* check_contiguity - Get information on physical memory associated with virtual
 * address.
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
int check_contiguity(unsigned long vaddr, pid_t pid, struct physmem *pmem, size_t size);

#endif // PARROT_VIRT2PHYS
