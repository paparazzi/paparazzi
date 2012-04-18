/* ***************************************************************************************************************

	crt.S						STARTUP  ASSEMBLY  CODE 
								-----------------------


	Module includes the interrupt vectors and start-up code.

  *************************************************************************************************************** */

/* use the four first sectors for bootloader */
.set  APP_START, 0x00004000

.set  IOPIN0,    0xE0028000
.set  IOPIN1,    0xE0028010
.set  PINSEL2,   0xE002C014

#ifdef PROC_AP
#define BOARD_CLASSIX
#endif
#ifdef PROC_FBW
#define BOARD_CLASSIX
#endif
#ifdef PROC_TINY
#define BOARD_TINY
#endif
#ifdef PROC_TINYJ
#define BOARD_TINYJ
#endif
#ifdef PROC_LPCH2148
#define BOARD_LPCH2148
#endif
#ifdef PROC_LOGGER
#define BOARD_LOGGER
#endif

#ifdef BOARD_TINYJ
/* TINY Jeremy old    MOSI P0.19 */
/* .set  BOOT_PIN,  0x00080000 */
/* TINY Jeremy old    SCK P0.17 */
.set BOOT_PIN,   0x00020000
#else
#ifdef BOARD_TINY
/* TINY old    LPC_SSEL P0.20 */
.set  BOOT_PIN,  0x00100000
#else
#ifdef BOARD_CLASSIX
/* CLASSIX old I2C_SDA  P0.03 */
.set  BOOT_PIN,  0x00000008
#else
#ifdef BOARD_LPCH2148
/* Olimex LPC-H2148 P1.24 */
.set  BOOT_PIN,  0x04000000
#else
#ifdef BOARD_LOGGER
/* SD-Logger P0.29 */
.set  BOOT_PIN,  0x20000000
#else
/* Vbus P0.23 */
.set  BOOT_PIN,  0x00800000
#endif
#endif
#endif
#endif
#endif

/* Stack Sizes */
.set  UND_STACK_SIZE, 0x00000040		/* stack for "undefined instruction" interrupts is 4 bytes  */
.set  ABT_STACK_SIZE, 0x00000040		/* stack for "abort" interrupts is 4 bytes                  */
.set  FIQ_STACK_SIZE, 0x00000040		/* stack for "FIQ" interrupts  is 4 bytes         			*/
.set  IRQ_STACK_SIZE, 0X00000040		/* stack for "IRQ" normal interrupts is 4 bytes    			*/
.set  SVC_STACK_SIZE, 0x00000400		/* stack for "SVC" supervisor mode is 4 bytes  				*/



/* Standard definitions of Mode bits and Interrupt (I & F) flags in PSRs (program status registers) */
.set  MODE_USR, 0x10            		/* Normal User Mode 										*/
.set  MODE_FIQ, 0x11            		/* FIQ Processing Fast Interrupts Mode 						*/
.set  MODE_IRQ, 0x12            		/* IRQ Processing Standard Interrupts Mode 					*/
.set  MODE_SVC, 0x13            		/* Supervisor Processing Software Interrupts Mode 			*/
.set  MODE_ABT, 0x17            		/* Abort Processing memory Faults Mode 						*/
.set  MODE_UND, 0x1B            		/* Undefined Processing Undefined Instructions Mode 		*/
.set  MODE_SYS, 0x1F            		/* System Running Priviledged Operating System Tasks  Mode	*/

.set  I_BIT, 0x80               		/* when I bit is set, IRQ is disabled (program status registers) */
.set  F_BIT, 0x40               		/* when F bit is set, FIQ is disabled (program status registers) */


.text
.arm

.global	Reset_Handler
.global _startup
.func   _startup

_startup:

# Exception Vectors

_vectors:       ldr     PC, Reset_Addr
                /*b _vectors*/
                ldr     PC, Undef_Addr
                ldr     PC, SWI_Addr
                ldr     PC, PAbt_Addr
                ldr     PC, DAbt_Addr
                nop							/* Reserved Vector (holds Philips ISP checksum) */
                ldr     PC, [PC,#-0xFF0]	 /* see page 71 of "Insiders Guide" Trevor Martin  */
                ldr     PC, FIQ_Addr                
                
Reset_Addr:     .word   Reset_Handler		/* defined in this module below  */
Undef_Addr:     .word   APP_START+0x04
SWI_Addr:       .word   APP_START+0x08
PAbt_Addr:      .word   APP_START+0x0C
DAbt_Addr:      .word   APP_START+0x10
Resvd_Addr:     .word   APP_START+0x14
IRQ_Addr:       .word   APP_START+0x18
FIQ_Addr:       .word   APP_START+0x1C

AppS_Addr:      .word   APP_START

# Reset Handler

Reset_Handler:  
#ifndef BOARD_LPCH2148
#ifndef BOARD_LOGGER
    			/* check status of the USB in P0.xx */
    			ldr   r0, =IOPIN0   /* load gpio addr */
    			ldr   r1,[r0]       /* get data */
    			ldr   r2, =BOOT_PIN /* get boot pin */
    			and   r1, r1, r2    /* clear all other bits */
    			cmp   r1, r2        /* compare bits */ 
    			beq   Bootloader    /* beq */
    			ldr   PC, AppS_Addr
#else
    			/* check status of the USB in P0.xx */
    			ldr   r0, =IOPIN0   /* load gpio addr */
    			ldr   r1,[r0]       /* get data */
    			ldr   r2, =BOOT_PIN /* get boot pin */
    			and   r1, r1, r2    /* clear all other bits */
    			cmp   r1, r2        /* compare bits */ 
    			bne   Bootloader    /* beq */
    			ldr   PC, AppS_Addr
#endif
#else
    			/* check status of the USB in P1.24 */
    			ldr   r0, =PINSEL2  /* load mode addr */
    			mov   r3, #8        /* Pins P1.25-16 are used as GPIO pins */
                str   r3, [r0]
    			ldr   r0, =IOPIN1   /* load gpio addr */
    			ldr   r1,[r0]       /* get data */
    			ldr   r2, =BOOT_PIN /* get boot pin */
    			ldr   r0, =PINSEL2  /* load mode addr */
    			mov   r3, #0        /* Pins P1.25-16 default */
                str   r3, [r0]
    			and   r1, r1, r2    /* clear all other bits */
    			cmp   r1, r2        /* compare bits */ 
    			beq   Bootloader    /* beq */
    			ldr   PC, AppS_Addr
#endif
            
				/* Setup a stack for each mode - note that this only sets up a usable stack
				for User mode.   Also each mode is setup with interrupts initially disabled. */
Bootloader:
    			ldr   r0, =_stack_end
    			msr   CPSR_c, #MODE_UND|I_BIT|F_BIT 	/* Undefined Instruction Mode  */
    			mov   sp, r0
    			sub   r0, r0, #UND_STACK_SIZE
    			msr   CPSR_c, #MODE_ABT|I_BIT|F_BIT 	/* Abort Mode */
    			mov   sp, r0
    			sub   r0, r0, #ABT_STACK_SIZE
    			msr   CPSR_c, #MODE_FIQ|I_BIT|F_BIT 	/* FIQ Mode */
    			mov   sp, r0	
   				sub   r0, r0, #FIQ_STACK_SIZE
    			msr   CPSR_c, #MODE_IRQ|I_BIT|F_BIT 	/* IRQ Mode */
    			mov   sp, r0
    			sub   r0, r0, #IRQ_STACK_SIZE
    			msr   CPSR_c, #MODE_SVC|I_BIT|F_BIT 	/* Supervisor Mode */
    			mov   sp, r0
    			sub   r0, r0, #SVC_STACK_SIZE
    			msr   CPSR_c, #MODE_SYS|I_BIT|F_BIT 	/* User Mode */
    			mov   sp, r0

				/* copy .data section (Copy from ROM to RAM) */
                ldr     R1, =_etext
                ldr     R2, =_data
                ldr     R3, =_edata
1:        		cmp     R2, R3
                ldrlo   R0, [R1], #4
                strlo   R0, [R2], #4
                blo     1b

				/* Clear .bss section (Zero init)  */
                mov     R0, #0
                ldr     R1, =_bss_start
                ldr     R2, =_bss_end
2:				cmp     R1, R2
                strlo   R0, [R1], #4
                blo     2b

				/* Enter the C code  */
                b       main

.endfunc
.end
