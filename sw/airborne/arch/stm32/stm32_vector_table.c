/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "stm32_vector_table.h"

#include <cmsis/stm32.h>

#include "stm32_exceptions.h"

#ifdef USE_SYS_TIME
#include "mcu_periph/sys_time.h"
#define SYS_TICK_IRQ_HANDLER sys_tick_irq_handler
#else
#define SYS_TICK_IRQ_HANDLER null_handler
#endif

#if defined USE_UART1 || OVERRIDE_UART1_IRQ_HANDLER
#include "mcu_periph/uart.h"
#define USART1_IRQ_HANDLER usart1_irq_handler
#else
#define USART1_IRQ_HANDLER null_handler
#endif

#if defined USE_UART2 || OVERRIDE_UART2_IRQ_HANDLER
#include "mcu_periph/uart.h"
#define USART2_IRQ_HANDLER usart2_irq_handler
#else
#define USART2_IRQ_HANDLER null_handler
#endif

#if defined USE_UART3 || OVERRIDE_UART3_IRQ_HANDLER
#include "mcu_periph/uart.h"
#define USART3_IRQ_HANDLER usart3_irq_handler
#else
#define USART3_IRQ_HANDLER null_handler
#endif

#if defined USE_UART5 || OVERRIDE_UART5_IRQ_HANDLER
#include "mcu_periph/uart.h"
#define USART5_IRQ_HANDLER usart5_irq_handler
#else
#define USART5_IRQ_HANDLER null_handler
#endif


#ifdef USE_I2C1
#include "mcu_periph/i2c_arch.h"
#define I2C1_EV_IRQ_HANDLER i2c1_ev_irq_handler
#define I2C1_ER_IRQ_HANDLER i2c1_er_irq_handler
#else
#define I2C1_EV_IRQ_HANDLER null_handler
#define I2C1_ER_IRQ_HANDLER null_handler
#endif

#ifdef USE_I2C2
#include "mcu_periph/i2c_arch.h"
#define I2C2_EV_IRQ_HANDLER i2c2_ev_irq_handler
#define I2C2_ER_IRQ_HANDLER i2c2_er_irq_handler
#else
#define I2C2_EV_IRQ_HANDLER null_handler
#define I2C2_ER_IRQ_HANDLER null_handler
#endif

#ifdef USE_SPI1_IRQ
extern void spi1_irq_handler(void);
#define SPI1_IRQ_HANDLER spi1_irq_handler
#else
#define SPI1_IRQ_HANDLER null_handler
#endif

#ifdef USE_SPI2_IRQ
extern void spi2_irq_handler(void);
#define SPI2_IRQ_HANDLER spi2_irq_handler
#else
#define SPI2_IRQ_HANDLER null_handler
#endif

#ifdef USE_EXTI0_IRQ
extern void exti0_irq_handler(void);
#define EXTI0_IRQ_HANDLER exti0_irq_handler
#else
#define EXTI0_IRQ_HANDLER null_handler
#endif

#ifdef USE_EXTI2_IRQ
extern void exti2_irq_handler(void);
#define EXTI2_IRQ_HANDLER exti2_irq_handler
#else
#define EXTI2_IRQ_HANDLER null_handler
#endif

#ifdef USE_EXTI3_IRQ
extern void exti3_irq_handler(void);
#define EXTI3_IRQ_HANDLER exti3_irq_handler
#else
#define EXTI3_IRQ_HANDLER null_handler
#endif

#ifdef USE_EXTI4_IRQ
extern void exti4_irq_handler(void);
#define EXTI4_IRQ_HANDLER exti4_irq_handler
#else
#define EXTI4_IRQ_HANDLER null_handler
#endif

#ifdef USE_EXTI9_5_IRQ
extern void exti9_5_irq_handler(void);
#define EXTI9_5_IRQ_HANDLER exti9_5_irq_handler
#else
#define EXTI9_5_IRQ_HANDLER null_handler
#endif

#ifdef USE_EXTI15_10_IRQ
extern void exti15_10_irq_handler(void);
#define EXTI15_10_IRQ_HANDLER exti15_10_irq_handler
#else
#define EXTI15_10_IRQ_HANDLER null_handler
#endif


#ifdef USE_DMA1_C2_IRQ
extern void dma1_c2_irq_handler(void);
#define DMA1_C2_IRQ_HANDLER dma1_c2_irq_handler
#else
#define DMA1_C2_IRQ_HANDLER null_handler
#endif

#ifdef USE_DMA1_C4_IRQ
extern void dma1_c4_irq_handler(void);
#define DMA1_C4_IRQ_HANDLER dma1_c4_irq_handler
#else
#define DMA1_C4_IRQ_HANDLER null_handler
#endif

#ifdef USE_ADC1_2_IRQ_HANDLER
extern void adc1_2_irq_handler(void);
#define ADC1_2_IRQ_HANDLER adc1_2_irq_handler
#else
#define ADC1_2_IRQ_HANDLER null_handler
#endif

#ifdef USE_TIM2_IRQ
extern void tim2_irq_handler(void);
#define TIM2_IRQ_HANDLER tim2_irq_handler
#else
#define TIM2_IRQ_HANDLER null_handler
#endif

#ifdef USE_TIM6_IRQ
extern void tim6_irq_handler(void);
#define TIM6_IRQ_HANDLER tim6_irq_handler
#else
#define TIM6_IRQ_HANDLER null_handler
#endif

#ifdef USE_USB_HP_CAN1_TX_IRQ
extern void usb_hp_can1_tx_irq_handler(void);
#define USB_HP_CAN1_TX_IRQ_HANDLER usb_hp_can1_tx_irq_handler
#else
#define USB_HP_CAN1_TX_IRQ_HANDLER null_handler
#endif

#ifdef USE_USB_LP_CAN1_RX0_IRQ
extern void usb_lp_can1_rx0_irq_handler(void);
#define USB_LP_CAN1_RX0_IRQ_HANDLER usb_lp_can1_rx0_irq_handler
#else
#define USB_LP_CAN1_RX0_IRQ_HANDLER null_handler
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
    EXTI0_IRQ_HANDLER,        /* exti0_irq_handler */
    null_handler,             /* exti1_irq_handler */
    EXTI2_IRQ_HANDLER,        /* exti2_irq_handler */
    EXTI3_IRQ_HANDLER,        /* exti3_irq_handler */
    EXTI4_IRQ_HANDLER,        /* exti4_irq_handler */
    null_handler,             /* dma1_channel1_irq_handler */
    DMA1_C2_IRQ_HANDLER,      /* dma1_channel2_irq_handler */
    null_handler,             /* dma1_channel3_irq_handler */
    DMA1_C4_IRQ_HANDLER,      /* dma1_channel4_irq_handler */
    null_handler,             /* dma1_channel5_irq_handler */
    null_handler,             /* dma1_channel6_irq_handler */
    null_handler,             /* dma1_channel7_irq_handler */
    ADC1_2_IRQ_HANDLER,       /* adc1_2_irq_handler */
    USB_HP_CAN1_TX_IRQ_HANDLER, /* usb_hp_can_tx_irq_handler */
    USB_LP_CAN1_RX0_IRQ_HANDLER, /* usb_lp_can_rx0_irq_handler */
    null_handler,             /* can_rx1_irq_handler */
    null_handler,             /* can_sce_irq_handler */
    EXTI9_5_IRQ_HANDLER,      /* exti9_5_irq_handler */
    null_handler,             /* tim1_brk_irq_handler */
    null_handler,             /* tim1_up_irq_handler */
    null_handler,             /* tim1_trg_com_irq_handler */
    null_handler,             /* tim1_cc_irq_handler */
    TIM2_IRQ_HANDLER,         /* tim2_irq_handler */
    null_handler,             /* tim3_irq_handler */
    null_handler,             /* tim4_irq_handler */
    I2C1_EV_IRQ_HANDLER,      /* i2c1_ev_irq_handler */
    I2C1_ER_IRQ_HANDLER,      /* i2c1_er_irq_handler */
    I2C2_EV_IRQ_HANDLER,      /* i2c2_ev_irq_handler */
    I2C2_ER_IRQ_HANDLER,      /* i2c2_er_irq_handler */
    SPI1_IRQ_HANDLER,         /* spi1_irq_handler */
    SPI2_IRQ_HANDLER,         /* spi2_irq_handler */
    USART1_IRQ_HANDLER,       /* usart1_irq_handler */
    USART2_IRQ_HANDLER,       /* usart2_irq_handler */
    USART3_IRQ_HANDLER,       /* usart3_irq_handler */
    EXTI15_10_IRQ_HANDLER,    /* exti15_10_irq_handler */
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
    USART5_IRQ_HANDLER,       /* uart5_irq_handler */
    TIM6_IRQ_HANDLER,         /* tim6_irq_handler */
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

//extern int main(int argc, char** argv);
extern void main( void);

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

/* FIXME: look deeper into what that is doing
 *
 */

void assert_param(void);

void assert_param(void){

}

