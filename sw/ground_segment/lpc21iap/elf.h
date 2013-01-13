/*
elf.h - various ELF informations collected from libelf
Copyright (C) 1995 - 1998 Michael Riepe

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Library General Public
License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Library General Public License for more details.

You should have received a copy of the GNU Library General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/* @(#) $Id$ */

/*
 * ELF header
 */
#define EI_NIDENT	16

typedef struct {
    unsigned char	e_ident[EI_NIDENT];
    unsigned short	e_type;
    unsigned short	e_machine;
    unsigned int	e_version;
    unsigned int	e_entry;
    unsigned int	e_phoff;
    unsigned int	e_shoff;
    unsigned int	e_flags;
    unsigned short	e_ehsize;
    unsigned short	e_phentsize;
    unsigned short	e_phnum;
    unsigned short	e_shentsize;
    unsigned short	e_shnum;
    unsigned short	e_shstrndx;
} Elf32_Ehdr;

/*
 * e_ident
 */
#define EI_MAG0		0
#define EI_MAG1		1
#define EI_MAG2		2
#define EI_MAG3		3
#define EI_CLASS	4
#define EI_DATA		5
#define EI_VERSION	6
#define EI_OSABI	7
#define EI_ABIVERSION	8
#define EI_PAD		9

#define ELFMAG0		0x7f
#define ELFMAG1		'E'
#define ELFMAG2		'L'
#define ELFMAG3		'F'
#define ELFMAG		"\177ELF"
#define SELFMAG		4

/*
 * e_ident[EI_CLASS]
 */
#define ELFCLASSNONE    0
#define ELFCLASS32      1
#define ELFCLASS64      2
#define ELFCLASSNUM     3

/*
 * e_ident[EI_DATA]
 */
#define ELFDATANONE	0
#define ELFDATA2LSB	1
#define ELFDATA2MSB	2
#define ELFDATANUM	3

/*
 * e_ident[EI_OSABI]
 */
#define ELFOSABI_NONE		0	/* No extensions or unspecified */
#define ELFOSABI_SYSV		ELFOSABI_NONE
#define ELFOSABI_HPUX		1	/* Hewlett-Packard HP-UX */
#define ELFOSABI_NETBSD		2	/* NetBSD */
#define ELFOSABI_LINUX		3	/* Linux */
#define ELFOSABI_SOLARIS	6	/* Sun Solaris */
#define ELFOSABI_AIX		7	/* AIX */
#define ELFOSABI_IRIX		8	/* IRIX */
#define ELFOSABI_FREEBSD	9	/* FreeBSD */
#define ELFOSABI_TRU64		10	/* Compaq TRU64 UNIX */
#define ELFOSABI_MODESTO	11	/* Novell Modesto */
#define ELFOSABI_OPENBSD	12	/* Open BSD */
#define ELFOSABI_OPENVMS	13	/* Open VMS */
#define ELFOSABI_NSK		14	/* Hewlett-Packard Non-Stop Kernel */
#define ELFOSABI_AROS	 	15 	/* Amiga Research OS */
/* these are probably obsolete: */
#define ELFOSABI_ARM		97	/* ARM */
#define ELFOSABI_STANDALONE	255	/* standalone (embedded) application */

/*
 * e_type
 */
#define ET_NONE		0
#define ET_REL		1
#define ET_EXEC		2
#define ET_DYN		3
#define ET_CORE		4
#define ET_NUM		5
#define ET_LOOS		0xfe00
#define ET_HIOS		0xfeff
#define ET_LOPROC	0xff00
#define ET_HIPROC	0xffff

/*
 * e_machine
 */
#define EM_NONE		0	/* No machine */
#define EM_M32		1	/* AT&T WE 32100 */
#define EM_SPARC	2	/* SPARC */
#define EM_386		3	/* Intel 80386 */
#define EM_68K		4	/* Motorola 68000 */
#define EM_88K		5	/* Motorola 88000 */
#define EM_486		6	/* Intel i486 (DO NOT USE THIS ONE) */
#define EM_860		7	/* Intel 80860 */
#define EM_MIPS		8	/* MIPS I Architecture */
#define EM_S370		9	/* IBM System/370 Processor */
#define EM_MIPS_RS3_LE	10	/* MIPS RS3000 Little-endian */
#define EM_SPARC64	11	/* SPARC 64-bit */
#define EM_PARISC	15	/* Hewlett-Packard PA-RISC */
#define EM_VPP500	17	/* Fujitsu VPP500 */
#define EM_SPARC32PLUS	18	/* Enhanced instruction set SPARC */
#define EM_960		19	/* Intel 80960 */
#define EM_PPC		20	/* PowerPC */
#define EM_PPC64	21	/* 64-bit PowerPC */
#define EM_S390		22	/* IBM System/390 Processor */
#define EM_V800		36	/* NEC V800 */
#define EM_FR20		37	/* Fujitsu FR20 */
#define EM_RH32		38	/* TRW RH-32 */
#define EM_RCE		39	/* Motorola RCE */
#define EM_ARM		40	/* Advanced RISC Machines ARM */
#define EM_ALPHA	41	/* Digital Alpha */
#define EM_SH		42	/* Hitachi SH */
#define EM_SPARCV9	43	/* SPARC Version 9 */
#define EM_TRICORE	44	/* Siemens TriCore embedded processor */
#define EM_ARC		45	/* Argonaut RISC Core, Argonaut Technologies Inc. */
#define EM_H8_300	46	/* Hitachi H8/300 */
#define EM_H8_300H	47	/* Hitachi H8/300H */
#define EM_H8S		48	/* Hitachi H8S */
#define EM_H8_500	49	/* Hitachi H8/500 */
#define EM_IA_64	50	/* Intel IA-64 processor architecture */
#define EM_MIPS_X	51	/* Stanford MIPS-X */
#define EM_COLDFIRE	52	/* Motorola ColdFire */
#define EM_68HC12	53	/* Motorola M68HC12 */
#define EM_MMA		54	/* Fujitsu MMA Multimedia Accelerator */
#define EM_PCP		55	/* Siemens PCP */
#define EM_NCPU		56	/* Sony nCPU embedded RISC processor */
#define EM_NDR1		57	/* Denso NDR1 microprocessor */
#define EM_STARCORE	58	/* Motorola Star*Core processor */
#define EM_ME16		59	/* Toyota ME16 processor */
#define EM_ST100	60	/* STMicroelectronics ST100 processor */
#define EM_TINYJ	61	/* Advanced Logic Corp. TinyJ embedded processor family */
#define EM_X86_64	62	/* AMD x86-64 architecture */
#define EM_PDSP		63	/* Sony DSP Processor */
#define EM_FX66		66	/* Siemens FX66 microcontroller */
#define EM_ST9PLUS	67	/* STMicroelectronics ST9+ 8/16 bit microcontroller */
#define EM_ST7		68	/* STMicroelectronics ST7 8-bit microcontroller */
#define EM_68HC16	69	/* Motorola MC68HC16 Microcontroller */
#define EM_68HC11	70	/* Motorola MC68HC11 Microcontroller */
#define EM_68HC08	71	/* Motorola MC68HC08 Microcontroller */
#define EM_68HC05	72	/* Motorola MC68HC05 Microcontroller */
#define EM_SVX		73	/* Silicon Graphics SVx */
#define EM_ST19		74	/* STMicroelectronics ST19 8-bit microcontroller */
#define EM_VAX		75	/* Digital VAX */
#define EM_CRIS		76	/* Axis Communications 32-bit embedded processor */
#define EM_JAVELIN	77	/* Infineon Technologies 32-bit embedded processor */
#define EM_FIREPATH	78	/* Element 14 64-bit DSP Processor */
#define EM_ZSP		79	/* LSI Logic 16-bit DSP Processor */
#define EM_MMIX		80	/* Donald Knuth's educational 64-bit processor */
#define EM_HUANY	81	/* Harvard University machine-independent object files */
#define EM_PRISM	82	/* SiTera Prism */
#define EM_AVR		83	/* Atmel AVR 8-bit microcontroller */
#define EM_FR30		84	/* Fujitsu FR30 */
#define EM_D10V		85	/* Mitsubishi D10V */
#define EM_D30V		86	/* Mitsubishi D30V */
#define EM_V850		87	/* NEC v850 */
#define EM_M32R		88	/* Mitsubishi M32R */
#define EM_MN10300	89	/* Matsushita MN10300 */
#define EM_MN10200	90	/* Matsushita MN10200 */
#define EM_PJ		91	/* picoJava */
#define EM_OPENRISC	92	/* OpenRISC 32-bit embedded processor */
#define EM_ARC_A5	93	/* ARC Cores Tangent-A5 */
#define EM_XTENSA	94	/* Tensilica Xtensa Architecture */
#define EM_VIDEOCORE	95	/* Alphamosaic VideoCore processor */
#define EM_TMM_GPP	96	/* Thompson Multimedia General Purpose Processor */
#define EM_NS32K	97	/* National Semiconductor 32000 series */
#define EM_TPC		98	/* Tenor Network TPC processor */
#define EM_SNP1K	99	/* Trebia SNP 1000 processor */
#define EM_ST200	100	/* STMicroelectronics (www.st.com) ST200 microcontroller */
#define EM_IP2K 	101 	/* Ubicom IP2xxx microcontroller family */
#define EM_MAX 		102 	/* MAX Processor */
#define EM_CR 		103 	/* National Semiconductor CompactRISC microprocessor */
#define EM_F2MC16 	104 	/* Fujitsu F2MC16 */
#define EM_MSP430 	105 	/* Texas Instruments embedded microcontroller msp430 */
#define EM_BLACKFIN	106 	/* Analog Devices Blackfin (DSP) processor */
#define EM_SE_C33 	107 	/* S1C33 Family of Seiko Epson processors */
#define EM_SEP 		108 	/* Sharp embedded microprocessor */
#define EM_ARCA 	109 	/* Arca RISC Microprocessor */
#define EM_UNICORE 	110 	/* Microprocessor series from PKU-Unity Ltd. and MPRC of Peking University */
#define EM_NUM		111

/*
 * e_ident[EI_VERSION], e_version
 */
#define EV_NONE		0
#define EV_CURRENT	1
#define EV_NUM		2

/*
 * e_flags
 */
#define EF_ARM_RELEXEC      0x01
#define EF_ARM_HASENTRY     0x02
#define EF_ARM_INTERWORK    0x04

/*
 * Program header
 */
typedef struct {
    unsigned int	p_type;
    unsigned int	p_offset;
    unsigned int	p_vaddr;
    unsigned int	p_paddr;
    unsigned int	p_filesz;
    unsigned int	p_memsz;
    unsigned int	p_flags;
    unsigned int	p_align;
} Elf32_Phdr;

/*
 * p_type
 */
#define PT_NULL		0
#define PT_LOAD		1
#define PT_DYNAMIC	2
#define PT_INTERP	3
#define PT_NOTE		4
#define PT_SHLIB	5
#define PT_PHDR		6
#define PT_TLS		7
#define PT_NUM		8
#define PT_LOOS		0x60000000
#define PT_HIOS		0x6fffffff
#define PT_LOPROC	0x70000000
#define PT_HIPROC	0x7fffffff
#define SHT_ARM_EXIDX	0x70000001

/*
 * p_flags
 */
#define PF_X		0x1
#define PF_W		0x2
#define PF_R		0x4
#define PF_MASKOS	0x0ff00000
#define PF_MASKPROC	0xf0000000

/*
 * Section header
 */
typedef struct {
    unsigned int	sh_name;
    unsigned int	sh_type;
    unsigned int	sh_flags;
    unsigned int	sh_addr;
    unsigned int	sh_offset;
    unsigned int	sh_size;
    unsigned int	sh_link;
    unsigned int	sh_info;
    unsigned int	sh_addralign;
    unsigned int	sh_entsize;
} Elf32_Shdr;

/*
 * sh_type
 */
#define SHT_NULL            0
#define SHT_PROGBITS        1
#define SHT_SYMTAB          2
#define SHT_STRTAB          3
#define SHT_RELA            4
#define SHT_HASH            5
#define SHT_DYNAMIC         6
#define SHT_NOTE            7
#define SHT_NOBITS          8
#define SHT_REL             9
#define SHT_SHLIB           10
#define SHT_DYNSYM          11
#define SHT_INIT_ARRAY      14
#define SHT_FINI_ARRA       15
#define SHT_PREINIT_ARRAY   16
#define SHT_GROUP           17
#define SHT_SYMTAB_SHNDX    18
#define SHT_NUM             19
#define SHT_LOOS            0x60000000
#define SHT_HIOS            0x6fffffff
#define SHT_LOPROC          0x70000000
#define SHT_HIPROC          0x7fffffff
#define SHT_LOUSER          0x80000000
#define SHT_HIUSER          0xffffffff

/*
 * sh_flags
 */
#define SHF_WRITE           0x1
#define SHF_ALLOC           0x2
#define SHF_EXECINSTR       0x4
#define SHF_MERGE           0x10
#define SHF_STRINGS         0x20
#define SHF_INFO_LINK       0x40
#define SHF_LINK_ORDER      0x80
#define SHF_OS_NONCONFORMING 0x100
#define SHF_GROUP           0x200
#define SHF_TLS             0x400
#define SHF_MASKOS          0x0ff00000
#define SHF_MASKPROC        0xf0000000

/* functions */

char ELFCheckHeader(unsigned char * binElf);
unsigned int ELFHdrFlags(unsigned char * binElf);
unsigned int ELFEntryAddr(unsigned char * binElf);
int ELFNoPSections(unsigned char * binElf);
unsigned int ELFTypePSection(unsigned char * binElf, int count);
unsigned int ELFFlagPSection(unsigned char * binElf, int count);
unsigned int ELFOffsPSection(unsigned char * binElf, int count);
unsigned int ELFAddrPSection(unsigned char * binElf, int count);
unsigned int ELFSizePSection(unsigned char * binElf, int count);

