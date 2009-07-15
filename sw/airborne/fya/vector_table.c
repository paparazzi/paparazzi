/*
 * Open-BLDC - Open BruschLess DC Motor Controller
 * Copyright (C) 2009 by Piotr Esden-Tempski <piotr@esden.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmsis/stm32.h>

#include "main.h"
#include "exceptions.h"

#include "vector_table.h"

#ifndef USE_SYS_TIME
#define SYS_TICK_IRQ_HANDLER null_handler
#else
#include "sys_time.h"
#define SYS_TICK_IRQ_HANDLER sys_tick_irq_handler
#endif

#ifndef USE_UART1
#define USART1_IRQ_HANDLER null_handler
#else
#include "uart.h"
#define USART1_IRQ_HANDLER usart1_irq_handler
#endif

#ifndef USE_UART2
#define USART2_IRQ_HANDLER null_handler
#else
#include "uart.h"
#define USART2_IRQ_HANDLER usart2_irq_handler
#endif

#ifndef USE_UART3
#define USART3_IRQ_HANDLER null_handler
#else
#include "uart.h"
#define USART3_IRQ_HANDLER usart3_irq_handler
#endif



/* addresses defined in the linker script */
extern unsigned long _etext;  /* end addr of .text section     */
extern unsigned long _sidata; /* init values for .data section */
extern unsigned long _sdata;  /* start addr of .data section   */
extern unsigned long _edata;  /* end addr of .data section     */
extern unsigned long _sbss;   /* start addr of .bss section    */
extern unsigned long _ebss;   /* end addr of .bss section      */
extern void _estack;          /* stack pointer init value      */

void reset_handler_stage1(void) __attribute__((__interrupt__));
void reset_handler_stage2(void);
void null_handler(void);

/* interrupt vector */
__attribute__ ((section(".isr_vector")))
void (* const vector_table[])(void) = {
    &_estack,              /* stack pointer init value*/
    reset_handler_stage1,  /* pc init value */
    nmi_exception,
    hard_fault_exception,
    mem_manage_exception,
    bus_fault_exception,
    usage_fault_exception,
    0, 0, 0, 0,               /* reserved */
    null_handler,             /* svc_handler */
    null_handler,             /* debug_monitor */
    0,                        /* reserved */
    null_handler,             /* pend_svc */
    SYS_TICK_IRQ_HANDLER,     /* sys_tick_handler, */
    null_handler,             /* wwdg_irq_handler */
    null_handler,             /* pvd_irq_handler */
    null_handler,             /* tamper_irq_handler */
    null_handler,             /* rtc_irq_handler */
    null_handler,             /* flash_irq_handler */
    null_handler,             /* rcc_irq_handler */
    null_handler,             /* exti0_irq_handler */
    null_handler,             /* exti1_irq_handler */
    null_handler,             /* exti2_irq_handler */
    null_handler,             /* exti3_irq_handler */
    null_handler,             /* exti4_irq_handler */
    null_handler,             /* dma1_channel1_irq_handler */
    null_handler,             /* dma1_channel2_irq_handler */
    null_handler,             /* dma1_channel3_irq_handler */
    null_handler,             /* dma1_channel4_irq_handler */
    null_handler,             /* dma1_channel5_irq_handler */
    null_handler,             /* dma1_channel6_irq_handler */
    null_handler,             /* dma1_channel7_irq_handler */
    null_handler,             /* adc1_2_irq_handler */
    null_handler,             /* usb_hp_can_tx_irq_handler */
    null_handler,             /* usb_lp_can_rx0_irq_handler */
    null_handler,             /* can_rx1_irq_handler */
    null_handler,             /* can_sce_irq_handler */
    null_handler,             /* exti9_5_irq_handler */
    null_handler,             /* tim1_brk_irq_handler */
    null_handler,             /* tim1_up_irq_handler */
    null_handler,             /* tim1_trg_com_irq_handler */
    null_handler,             /* tim1_cc_irq_handler */
    null_handler,             /* tim2_irq_handler */
    null_handler,             /* tim3_irq_handler */
    null_handler,             /* tim4_irq_handler */
    null_handler,             /* i2c1_ev_irq_handler */
    null_handler,             /* i2c1_er_irq_handler */
    null_handler,             /* i2c2_ev_irq_handler */
    null_handler,             /* i2c2_er_irq_handler */
    null_handler,             /* spi1_irq_handler */
    null_handler,             /* spi2_irq_handler */
    USART1_IRQ_HANDLER,       /* usart1_irq_handler */
    USART2_IRQ_HANDLER,       /* usart2_irq_handler */
    USART3_IRQ_HANDLER,       /* usart3_irq_handler */
    null_handler,             /* exti15_10_irq_handler */
    null_handler,             /* rtc_alarm_irq_handler */
    null_handler,             /* usb_wake_up_irq_handler */
    null_handler,             /* tim8_brk_irq_handler */
    null_handler,             /* tim8_up_irq_handler */
    null_handler,             /* tim8_trg_com_irq_handler */
    null_handler,             /* tim8_cc_irq_handler */
    null_handler,             /* adc3_irq_handler */
    null_handler,             /* fsmc_irq_handler */
    null_handler,             /* sdio_irq_handler */
    null_handler,             /* tim5_irq_handler */
    null_handler,             /* spi3_irq_handler */
    null_handler,             /* uart4_irq_handler */
    null_handler,             /* uart5_irq_handler */
    null_handler,             /* tim6_irq_handler */
    null_handler,             /* tim7_irq_handler */
    null_handler,             /* dma2_channel1_irq_handler */
    null_handler,             /* dma2_channel2_irq_handler */
    null_handler,             /* dma2_channel3_irq_handler */
    null_handler,             /* dma2_channel4_5_irq_handler */
};

/* Get's called directly after mcu reset */
void reset_handler_stage1(void){
    /* set stack align */
    SCB->CCR = SCB->CCR | SCB_CCR_STKALIGN;

    reset_handler_stage2();
}

void reset_handler_stage2(void){
    unsigned long *pul_src, *pul_dest;

    /* Copy the data segment initializers from flash to SRAM */
    pul_src = &_sidata;
    for(pul_dest = &_sdata; pul_dest < &_edata; ){
        *(pul_dest++) = *(pul_src++);
    }
    /* Zero fill the bss segment.  */
    for(pul_dest = &_sbss; pul_dest < &_ebss; ){
        *(pul_dest++) = 0;
    }

    /* Call the application's entry point.*/
    main();
}

void null_handler(void){
}

void assert_param(void){
}
