/*  $Id$
 *
 * lpc21iap, an USB download application for Philips LPC processors
 * Copyright (C) 2006  Martin Mueller <martinmm@pfump.org>
 *
 * This file is part of paparazzi.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "elf.h"

char ELFCheckHeader(unsigned char * binElf)
{
    Elf32_Ehdr * eHdr = (Elf32_Ehdr*) binElf;

    if ((eHdr->e_ident[EI_MAG0]    == ELFMAG0) &&
        (eHdr->e_ident[EI_MAG1]    == ELFMAG1) &&
        (eHdr->e_ident[EI_MAG2]    == ELFMAG2) &&
        (eHdr->e_ident[EI_MAG3]    == ELFMAG3) &&
        (eHdr->e_ident[EI_CLASS]   == ELFCLASS32) &&
        (eHdr->e_ident[EI_DATA]    == ELFDATA2LSB) &&
        (eHdr->e_ident[EI_VERSION] == EV_CURRENT) &&
        ((eHdr->e_ident[EI_OSABI]   == ELFOSABI_ARM) ||
         (eHdr->e_ident[EI_OSABI]   == ELFOSABI_NONE)) &&
        (eHdr->e_type              == ET_EXEC) &&
        (eHdr->e_machine           == EM_ARM) &&
        (eHdr->e_version           == EV_CURRENT))
    {
        return(1);
    }

   return(0);
}

unsigned int ELFHdrFlags(unsigned char * binElf)
{
    Elf32_Ehdr * eHdr = (Elf32_Ehdr*) binElf;

    return(eHdr->e_flags);
}

unsigned int ELFEntryAddr(unsigned char * binElf)
{
    Elf32_Ehdr * eHdr = (Elf32_Ehdr*) binElf;

    return(eHdr->e_entry);
}

/* program header functions */

int ELFNoPSections(unsigned char * binElf)
{
    Elf32_Ehdr * eHdr = (Elf32_Ehdr*) binElf;

    return(eHdr->e_phnum);
}

unsigned int ELFTypePSection(unsigned char * binElf, int count)
{
   Elf32_Ehdr * eHdr = (Elf32_Ehdr*) binElf;
   Elf32_Phdr * pHdr;

   pHdr = (Elf32_Phdr *) (binElf + eHdr->e_phoff);
   pHdr += count;

   return(pHdr->p_type);
}

unsigned int ELFFlagPSection(unsigned char * binElf, int count)
{
   Elf32_Ehdr * eHdr = (Elf32_Ehdr*) binElf;
   Elf32_Phdr * pHdr;

   pHdr = (Elf32_Phdr *) (binElf + eHdr->e_phoff);
   pHdr += count;

   return(pHdr->p_flags);
}
unsigned int ELFOffsPSection(unsigned char * binElf, int count)
{
   Elf32_Ehdr * eHdr = (Elf32_Ehdr*) binElf;
   Elf32_Phdr * pHdr;

   pHdr = (Elf32_Phdr *) (binElf + eHdr->e_phoff);
   pHdr += count;

   return(pHdr->p_offset);
}


unsigned int ELFAddrPSection(unsigned char * binElf, int count)
{
   Elf32_Ehdr * eHdr = (Elf32_Ehdr*) binElf;
   Elf32_Phdr * pHdr;

   pHdr = (Elf32_Phdr *) (binElf + eHdr->e_phoff);
   pHdr += count;

   return(pHdr->p_paddr);
}

unsigned int ELFSizePSection(unsigned char * binElf, int count)
{
   Elf32_Ehdr * eHdr = (Elf32_Ehdr*) binElf;
   Elf32_Phdr * pHdr;

   pHdr = (Elf32_Phdr *) (binElf + eHdr->e_phoff);
   pHdr += count;

   return(pHdr->p_filesz);
}

