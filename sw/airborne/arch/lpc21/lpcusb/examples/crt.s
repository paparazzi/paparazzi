# 1 "crt.S"
# 1 "<built-in>"
# 1 "<command line>"
# 1 "crt.S"
# 12 "crt.S"
.set UND_STACK_SIZE, 0x00000040
.set ABT_STACK_SIZE, 0x00000040
.set FIQ_STACK_SIZE, 0x00000040
.set IRQ_STACK_SIZE, 0X00000100
.set SVC_STACK_SIZE, 0x00000400




.set MODE_USR, 0x10
.set MODE_FIQ, 0x11
.set MODE_IRQ, 0x12
.set MODE_SVC, 0x13
.set MODE_ABT, 0x17
.set MODE_UND, 0x1B
.set MODE_SYS, 0x1F

.set I_BIT, 0x80
.set F_BIT, 0x40


.text
.arm

.global Reset_Handler
.global _startup
.func _startup

_startup:

# Exception Vectors

_vectors: ldr PC, Reset_Addr
                ldr PC, Undef_Addr
                ldr PC, SWI_Addr
                ldr PC, PAbt_Addr
                ldr PC, DAbt_Addr
                nop
                ldr PC, [PC,#-0xFF0]
                ldr PC, FIQ_Addr

Reset_Addr: .word Reset_Handler
Undef_Addr: .word UNDEF_Routine
SWI_Addr: .word SWI_Routine
PAbt_Addr: .word UNDEF_Routine
DAbt_Addr: .word UNDEF_Routine
IRQ_Addr: .word IRQ_Routine
FIQ_Addr: .word FIQ_Routine
                .word 0


# Reset Handler

Reset_Handler:




       ldr r0, =_stack_end
       msr CPSR_c, #MODE_UND|I_BIT|F_BIT
       mov sp, r0
       sub r0, r0, #UND_STACK_SIZE
       msr CPSR_c, #MODE_ABT|I_BIT|F_BIT
       mov sp, r0
       sub r0, r0, #ABT_STACK_SIZE
       msr CPSR_c, #MODE_FIQ|I_BIT|F_BIT
       mov sp, r0
       sub r0, r0, #FIQ_STACK_SIZE
       msr CPSR_c, #MODE_IRQ|I_BIT|F_BIT
       mov sp, r0
       sub r0, r0, #IRQ_STACK_SIZE
       msr CPSR_c, #MODE_SVC|I_BIT|F_BIT
       mov sp, r0
       sub r0, r0, #SVC_STACK_SIZE
       msr CPSR_c, #MODE_SYS|I_BIT|F_BIT
       mov sp, r0


                ldr R1, =_etext
                ldr R2, =_data
                ldr R3, =_edata
1: cmp R2, R3
                ldrlo R0, [R1], #4
                strlo R0, [R2], #4
                blo 1b


                mov R0, #0
                ldr R1, =_bss_start
                ldr R2, =_bss_end
2: cmp R1, R2
                strlo R0, [R1], #4
                blo 2b


                b main

.endfunc
.end
