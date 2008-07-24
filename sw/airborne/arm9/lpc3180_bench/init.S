/*********************************************************FileHeaderBegin******
 *
 * FILE:
 *     init.S
 *
 * REVISION:
 *
 * AUTHOR:
 *     (C) 2008, Dirk Behme, dirk.behme@gmail.com
 *
 * CREATED:
 *     29.06.2008
 *
 * DESCRIPTION:
 *     Basic initialization for Phytec LPC3180 board.
 *     Note: This code assumes that basic processor initialization is
 *           already done (e.g. SDRAM, clock etc. ). This can be done
 *           e.g. by bootloader (U-Boot). We only do basic system
 *	     initialization most probably not done by bootloader here.
 *
 * NOTES:
 *
 * MODIFIED:
 *
 * MEMORY LAYOUT:
 *
 *      The virtual memory layout as configured here by MMU:	
 *
 *      0x00000000
 *                  |---------------------------|
 *                  |                           |
 *                  ~                           ~
 *                  |                           |
 *                  |---------------------------|
 *      0x08000000  |           iRAM            |
 *                  |        RW cached 1MB      |
 *      0x0BFFFFFF  |---------------------------|
 *                  |                           |
 *                  ~                           ~
 *                  |                           |
 *                  |---------------------------|
 *      0x20000000  |          AHB ch 5         |
 *                  |        RW nocached 1MB    |
 *      0x200BFFFF  |---------------------------|
 *                  |                           |
 *                  ~                           ~
 *                  |                           |
 *                  |---------------------------|
 *      0x31000000  |          AHB ch 6         |
 *                  |        RW nocached 1MB    |
 *      0x310FFFFF  |---------------------------|
 *                  |                           |
 *                  ~                           ~
 *                  |                           |
 *                  |---------------------------|
 *      0x40000000  |          AHB ch 7         |
 *                  |        RW nocached 1MB    |
 *      0x400FFFFF  |---------------------------|
 *                  |                           |
 *                  ~                           ~
 *      0x80000000  |                           |
 *                  |---------------------------|
 *                  |                           |
 *                  |          SDRAM RW         |
 *                  |        cached 32MB        |
 *      0x81FFFFFF  |                           |
 *      0x82000000  |---------------------------|
 *                  |                           |
 *                  ~                           ~	
 *
 * LICENSE:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 **********************************************************FileHeaderEnd******/

/******************************************************************************
 * Defines
 *****************************************************************************/

#define STACK_TOP  0x82000000	/* Stack at top of 32MB SDRAM */ 

#define Mode_USR   0x10
#define Mode_FIQ   0x11
#define Mode_IRQ   0x12
#define Mode_SVC   0x13
#define Mode_ABT   0x17
#define Mode_UND   0x1B
#define Mode_SYS   0x1F

#define I_Bit      0x80		/* when I bit is set, IRQ is disabled */
#define F_Bit      0x40		/* when F bit is set, FIQ is disabled */

#define ZERO_BASE  0x04000000	/* DMA dummy space, reads as all zeros */
#define SRAM_BASE  0x08000000	/* 64k SRAM base */
#define UART5_BASE 0x40090000

/******************************************************************************
 * Entry code
 *****************************************************************************/

.text
.globl _start
_start:
	/* Exception vectors, located at 0x80000000,
	 * mapped from address 0 by MMU
	 */
	b	entry
	ldr	pc, _undefined_instruction
	ldr	pc, _software_interrupt
	ldr	pc, _prefetch_abort
	ldr	pc, _data_abort
	ldr	pc, _not_used
	ldr	pc, _irq
	mov     r3, #'F'	/* FIQ */
	b	error
	
_undefined_instruction:	.word undefined_instruction
_software_interrupt:	.word software_interrupt
_prefetch_abort:	.word prefetch_abort
_data_abort:		.word data_abort
_not_used:		.word not_used
_irq:			.word irq_handler
	
entry:
	/* Make sure we are in SVC mode and interrupts are off */
	msr     cpsr_c, #(Mode_SVC| I_Bit | F_Bit)

	/* Print a character */
	mov     r0, #'P'
	bl	uart5_putc
	
	/* flush I/D caches and TLB, drain write buffer */
	mov	r0, #0
	mcr	p15, 0, r0, c7, c7, 0
	mcr	p15, 0, r0, c8, c7, 0

	/* Print a character */
	mov     r0, #'a'
	bl	uart5_putc

	/* disable MMU and caches */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #0x00002300	/* clear bits 13, 9:8 (--V- --RS) */
	bic	r0, r0, #0x00000087	/* clear bits 7, 2:0 (B--- -CAM) */
	orr	r0, r0, #0x00000002	/* set bit 2 (A) Align */
	orr	r0, r0, #0x00001000	/* set bit 12 (I) I-Cache */
	mcr	p15, 0, r0, c1, c0, 0

	/* Print a character */
	mov     r0, #'p'
	bl	uart5_putc
	
	/* Initialize some registers to zero */
	ldr	r0, =ZERO_BASE
	ldmia	r0!, {r2, r3, r4, r5, r6, r7, r8, r9}

	/* Set TTB. For speed use internal SRAM */
	ldr	r0, =SRAM_BASE
	mcr	p15, 0, r0, c2, c0, 0	/* write TTB register */
	
	/* Clear 16kB of first level MMU table */	
	add	r1, r0, #(16*1024)
clear_l1:	
	stmia	r0!, {r2, r3, r4, r5, r6, r7, r8, r9}
	cmp	r0, r1
	ble	clear_l1
	
	/* Clear .bss section, we reuse zero's loaded above */
	ldr	r0, =_bss_start
	ldr	r1, =_bss_end
	cmp	r0, r1
	beq	start_main		/* Skip clear on empty bss */

bss_loop:
	/* Yes, by doing this, most probably we write over the end,
	 * but after the .bss section there is the heap not used yet.
	 * Do this to be able to use fast stmia instruction
	 */
	stmia	r0!, {r2, r3, r4, r5, r6, r7, r8, r9}
	cmp	r0, r1
	ble	bss_loop

	/* Print a character */
	mov     r0, #'a'
	bl	uart5_putc

	/* Set MMU table entries */
	ldr	r0, =SRAM_BASE
	ldr	r1, =0x80000C1E	/* Vectors: 0x00000000 -> 0x80000000 */
	str	r1, [r0]
	ldr	r1, =0x08000C1E	/* 1MB IRAM: 1:1 cachable bufferable */
	str	r1, [r0, #0x200]
	ldr	r1, =0x20000C12	/* 1MB AHB5: 1:1 non-cachable non-bufferable */
	str	r1, [r0, #0x800]
	ldr	r1, =0x31000C12	/* 1MB AHB6: 1:1 non-cachable non-bufferable */
	str	r1, [r0, #0xC00]
	ldr	r1, =0x40000C12	/* 1MB AHB7: 1:1 non-cachable non-bufferable */
	add	r0, r0, #0x1000
	str	r1, [r0]
	ldr	r1, =0x80000C1E	/* 32MB SDRAM: 1:1 cachable bufferable */
	ldr     r2, =0x00100000
	add	r0, r0, #0x1000
	str	r1, [r0, #0x00]
	add     r1, r1, r2
	str	r1, [r0, #0x04]
	add     r1, r1, r2
	str	r1, [r0, #0x08]
	add     r1, r1, r2
	str	r1, [r0, #0x0C]
	add     r1, r1, r2
	str	r1, [r0, #0x10]
	add     r1, r1, r2
	str	r1, [r0, #0x14]
	add     r1, r1, r2
	str	r1, [r0, #0x18]
	add     r1, r1, r2
	str	r1, [r0, #0x1C]
	add     r1, r1, r2
	str	r1, [r0, #0x20]
	add     r1, r1, r2
	str	r1, [r0, #0x24]
	add     r1, r1, r2
	str	r1, [r0, #0x28]
	add     r1, r1, r2
	str	r1, [r0, #0x2C]
	add     r1, r1, r2
	str	r1, [r0, #0x30]
	add     r1, r1, r2
	str	r1, [r0, #0x34]
	add     r1, r1, r2
	str	r1, [r0, #0x38]
	add     r1, r1, r2
	str	r1, [r0, #0x3C]
	add     r1, r1, r2
	str	r1, [r0, #0x40]
	add     r1, r1, r2
	str	r1, [r0, #0x44]
	add     r1, r1, r2
	str	r1, [r0, #0x48]
	add     r1, r1, r2
	str	r1, [r0, #0x4C]
	add     r1, r1, r2
	str	r1, [r0, #0x50]
	add     r1, r1, r2
	str	r1, [r0, #0x54]
	add     r1, r1, r2
	str	r1, [r0, #0x58]
	add     r1, r1, r2
	str	r1, [r0, #0x5C]
	add     r1, r1, r2
	str	r1, [r0, #0x60]
	add     r1, r1, r2
	str	r1, [r0, #0x64]
	add     r1, r1, r2
	str	r1, [r0, #0x68]
	add     r1, r1, r2
	str	r1, [r0, #0x6C]
	add     r1, r1, r2
	str	r1, [r0, #0x70]
	add     r1, r1, r2
	str	r1, [r0, #0x74]
	add     r1, r1, r2
	str	r1, [r0, #0x78]
	add     r1, r1, r2
	str	r1, [r0, #0x7C]
	add     r1, r1, r2

	/* Print a character */
	mov     r0, #'r'
	bl	uart5_putc

	/* Set domain access configuration */
	ldr	r0, =0xFFFFFFFF		/* manager, accesses are not checked */
	mcr	p15, 0, r0, c3, c0, 0

	/* Print a character */
	mov     r0, #'a'
	bl	uart5_putc
	
		
	/* Enable I & D cache and MMU */
	mrc	p15, 0, r0, c1, c0, 0
	orr	r0, r0, #0x00000005	/* set bit 2 (C) D-Cache and */
					/* bit 0 (M) MMU on */
	mcr	p15, 0, r0, c1, c0, 0
	nop
	nop
	nop

	/* Print a character */
	mov     r0, #'z'
	bl	uart5_putc	

	/* Setup stacks */
	ldr     r0, =STACK_TOP
	msr     cpsr_c, #(Mode_UND | I_Bit | F_Bit)	/* Undef mode */
        mov     sp, r0
	
        sub     r0, r0, #0x400
	msr     cpsr_c, #(Mode_ABT | I_Bit | F_Bit)	/* Abort mode */
	mov	sp, r0

	sub     r0, r0, #0x400
	msr     cpsr_c, #(Mode_FIQ | I_Bit | F_Bit)	/* FIQ mode */
	mov	sp, r0

	sub	r0, r0, #0x400
	msr     cpsr_c, #(Mode_IRQ | I_Bit | F_Bit)	/* IRQ mode */
	mov     sp, r0
	
	sub     r0, r0, #0x400
	msr     cpsr_c, #(Mode_SYS | I_Bit | F_Bit)	/* SYS mode */
	mov     sp, r0

	sub     r0, r0, #0x400
	msr     cpsr_c, #(Mode_SVC| F_Bit)	/* SVC mode, interrupts on */
	mov     sp, r0

	/* Print a character */
	mov     r0, #'z'
	bl	uart5_putc

	/* Enable VFP */
	mov     r0, #(1<<30)
        fmxr    fpexc, r0

	/* Print a character */
	mov     r0, #'i'
	bl	uart5_putc

	/* Jump to C main */
start_main:
	bl	main

/******************************************************************************
 * Helper functions
 *****************************************************************************/

	/* Return from main, stop in infinite loop */
.globl finished
finished:
	b	finished
	
	/* Output one byte to UART5 in polling mode */
	/* UART5 initialization is assumed to be done by U-Boot */
.globl uart5_putc
uart5_putc:
	ldr	r2,=UART5_BASE
putc:	ldrb	r1,[r2,#0x14]
	tst	r1,#0x20 
	beq	putc
	strb	r0,[r2]
	mov     pc, lr

	/* Return recent stack pointer */
.globl get_sp
get_sp:
	mov	r0, sp
	mov	pc, lr

/******************************************************************************
 * Exception handlers
 *****************************************************************************/

irq_handler:
	stmfd   sp!,{r0-r4,r12,lr}
	bl	dispatch_irq 
	ldmfd   sp!,{r0-r4,r12,lr}
	subs    pc,lr,#4 		


undefined_instruction:
	mov     r3, #'U'
	b	error

software_interrupt:
	mov     r3, #'S'
	b	error

prefetch_abort:
	mov     r3, #'P'
	b	error

data_abort:
	mov     r3, #'D'
	mrc	p15, 0, r4, c6, c0, 0		/* get fault addr in r4 */
	b	error

not_used:
	mov     r3, #'N'
	b	error

	/* Exception error handler */
error:
	mov     r0, #'E'
	bl	uart5_putc
	mov     r0, #'r'
	bl	uart5_putc
	mov     r0, #'r'
	bl	uart5_putc
	mov     r0, #'o'
	bl	uart5_putc
	mov     r0, #'r'
	bl	uart5_putc
	mov     r0, #':'
	bl	uart5_putc
	mov     r0, #' '
	bl	uart5_putc
	mov	r0, r3		/* Get error code */
	bl	uart5_putc
	/* Data abort? Print fault address */
	cmp	r0, #'D'
	bne	error_stop
	mov     r0, #' '
	bl	uart5_putc
	mov	r0, r4
	bl	print_int
error_stop:
	b	error_stop

/* End of init.S */
