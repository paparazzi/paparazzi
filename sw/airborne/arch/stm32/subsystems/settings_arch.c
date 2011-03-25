#include "subsystems/settings.h"

#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/crc.h>

struct FlashInfo {
    uint32_t addr;
    uint32_t total_size;
    uint32_t page_nr;
    uint32_t page_size;
};


static uint32_t pflash_checksum(uint32_t ptr, uint32_t size);
static int32_t flash_detect(struct FlashInfo* flash);
static int32_t pflash_program_bytes(struct FlashInfo* flash,
				     uint32_t src,
				     uint32_t size,
				     uint32_t chksum);

#define FLASH_BEGIN 0x08000000
#define FLASH_SIZE  0x1FFFF7E0
#define FSIZ        8
#define FCHK        4


static uint32_t pflash_checksum(uint32_t ptr, uint32_t size) {
  uint32_t i;

  /* reset crc */
  CRC_CR = CRC_CR_RESET;

  if (ptr % 4) {
    /* calc in 8bit chunks */
    for (i=0; i<(size & ~3); i+=4) {
      CRC_DR = (*(uint8_t*) (ptr+i)) |
               (*(uint8_t*) (ptr+i+1)) << 8 |
               (*(uint8_t*) (ptr+i+2)) << 16 |
               (*(uint8_t*) (ptr+i+3)) << 24;
    }
  } else {
    /* calc in 32bit */
    for (i=0; i<(size & ~3); i+=4) {
      CRC_DR = *(uint32_t*) (ptr+i);
    }
  }

  /* remaining bytes */
  switch (size % 4) {
    case 1:
      CRC_DR = *(uint8_t*) (ptr+i);
      break;
    case 2:
      CRC_DR = (*(uint8_t*) (ptr+i)) |
               (*(uint8_t*) (ptr+i+1)) << 8;
      break;
    case 3:
      CRC_DR = (*(uint8_t*) (ptr+i)) |
               (*(uint8_t*) (ptr+i+1)) << 8 |
               (*(uint8_t*) (ptr+i+2)) << 16;
      break;
  default:
    break;
  }

  return CRC_DR;
}
  


static int32_t flash_detect(struct FlashInfo* flash) {

  flash->total_size = (MMIO32(FLASH_SIZE) * 0x400)&0x00FFFFFF;

  switch (flash->total_size) {
    /* low density */
    case 0x00004000: /* 16 kBytes */
    case 0x00008000: /* 32 kBytes */
    /* medium density, e.g. STM32F103RBT6 (Olimex STM32-H103) */
    case 0x00010000: /* 64 kBytes */
    case 0x00200000: /* 128 kBytes */
    {
      flash->page_size = 0x400;
      break;
    }
    /* high density, e.g. STM32F103RE (Joby Lisa/M, Lisa/L) */
    case 0x00040000: /* 256 kBytes */
    case 0x00080000: /* 512 kBytes */
    /* XL density */
    case 0x000C0000: /* 768 kBytes */
    case 0x00100000: /* 1 MByte */
    {
      flash->page_size = 0x800;
      break;
    }
    default: {return -1;}
  }
  
  flash->page_nr = (flash->total_size / flash->page_size) - 1;
  flash->addr = FLASH_BEGIN + flash->page_nr * flash->page_size;
  
  return 0;
}

// (gdb) p *flash
// $1 = {addr = 134739968, total_size = 524288, page_nr = 255, page_size = 2048}
//              0x807F800             0x80000
static int32_t pflash_program_bytes(struct FlashInfo* flash,
				    uint32_t   src,
				    uint32_t   size,
				    uint32_t   chksum) {
  uint32_t i;

  /* erase */
  flash_unlock();
  flash_erase_page(flash->addr); 
  flash_lock();
  
  /* verify erase */
  for (i=0; i<flash->page_size; i+=4) {
    if ((*(uint32_t*) (flash->addr + i)) != 0xFFFFFFFF) return -1;
  }


  flash_unlock();
  /* write full 16 bit words */
  for (i=0; i<(size & ~1); i+=2) {
    flash_program_half_word(flash->addr+i, 
        (uint16_t)(*(uint8_t*)(src+i) | (*(uint8_t*)(src+i+1)) << 8));
  }
  /* fill bytes with a zero */
  if (size & 1) {
    flash_program_half_word(flash->addr+i, (uint16_t)(*(uint8_t*)(src+i)));
  }
  /* write size */
  flash_program_half_word(flash->addr+flash->page_size-FSIZ, 
                          (uint16_t)(size & 0xFFFF));
  flash_program_half_word(flash->addr+flash->page_size-FSIZ+2, 
                          (uint16_t)((size >> 16) & 0xFFFF));
  /* write checksum */
  flash_program_half_word(flash->addr+flash->page_size-FCHK, 
                          (uint16_t)(chksum & 0xFFFF));
  flash_program_half_word(flash->addr+flash->page_size-FCHK+2, 
                          (uint16_t)((chksum >> 16) & 0xFFFF));
  flash_lock();
  
  /* verify data */
  for (i=0; i<size; i++) {
    if ((*(uint8_t*) (flash->addr+i)) != (*(uint8_t*) (src+i))) return -2;
  }
  if (*(uint32_t*) (flash->addr+flash->page_size-FSIZ) != size) return -3;
  if (*(uint32_t*) (flash->addr+flash->page_size-FCHK) != chksum) return -4;
  
  return 0;
}


int32_t persistent_write(uint32_t ptr, uint32_t size) {
  struct FlashInfo flash_info;
  if (flash_detect(&flash_info)) return -1;
  if ((size > flash_info.page_size-FSIZ) || (size == 0)) return -2;
  
  return pflash_program_bytes(&flash_info, 
                              ptr,
                              size, 
                              pflash_checksum(ptr, size));
}

int32_t persistent_read(uint32_t ptr, uint32_t size) {
  struct FlashInfo flash;
  uint32_t i;

  /* check parameters */  
  if (flash_detect(&flash)) return -1;
  if ((size > flash.page_size-FSIZ) || (size == 0)) return -2;
  
  /* check consistency */
  if (size != *(uint32_t*)(flash.addr+flash.page_size-FSIZ)) return -3;
  if (pflash_checksum(flash.addr, size) != 
      *(uint32_t*)(flash.addr+flash.page_size-FCHK))
    return -4;
  
  /* copy data */  
  for (i=0; i<size; i++) {
    *(uint8_t*) (ptr+i) = *(uint8_t*) (flash.addr+i);
  }
                                
  return 0;
}
