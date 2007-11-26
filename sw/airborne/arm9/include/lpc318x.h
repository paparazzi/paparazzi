/******************************************************************************
 *   LPC318X.h:  Header file for Philips LPC318x Family Microprocessors
 *   The header file is the super set of all hardware definition of the 
 *   peripherals for the LPC318x family microprocessor.
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.

 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/

#ifndef __LPC318x_H
#define __LPC318x_H


/* System Control */
#define SYSCTRL_BASE_ADDR	0x40004000
#define BOOT_MAP	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x14))


/* Clock and Power Control */
#define CLK_PM_BASE_ADDR	0x40004000
#define PWR_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x44))
#define OSC_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x4C))
#define SYSCLK_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x50))
#define PLL397_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x48))	
#define HCLKPLL_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x58))
#define HCLKDIV_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x40))
#define TEST_CLK	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xA4))
#define AUTOCLK_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xEC))

#define START_ER_INT	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x20))
#define START_ER_PIN	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x30))
#define START_RSR_INT	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x24))
#define START_RSR_PIN	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x34))
#define START_SR_INT	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x28))
#define START_SR_PIN	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x38))
#define START_APR_INT	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x2C))
#define START_APR_PIN	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x3C))

#define DMACLK_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xE8))
#define UARTCLK_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xE4))
#define USBCLK_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x64))
#define MS_CTRL		(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x80))
#define I2CCLK_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xAC))
#define KEYCLK_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xB0))
#define ADCLK_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xB4))
#define PWMCLK_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xB8))
#define TIMCLK_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xBC))
#define SPI_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xC4))
#define FLASHCLK_CTRL	(*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xC8))


/* SDRAM Control register */
#define SDRAM_CLK_BASE_ADDR	0x40004000
#define SDRAMCLK_CTRL	(*(volatile unsigned long *)(SDRAM_CLK_BASE_ADDR + 0x68))

#define SDRAM_BASE_ADDR		0x31080000
#define MPMCControl	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x00))
#define MPMCStatus	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x04))
#define MPMCConfig	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x08))
#define MPMCDynamicControl	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x20))
#define MPMCDynamicRefresh	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x24))
#define MPMCDynamicReadConfig	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x28))
#define MPMCDynamicRP	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x30))
#define MPMCDynamicRAS	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x34))
#define MPMCDynamicSREX	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x38))
#define MPMCDynamicWR	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x44))
#define MPMCDynamicRC	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x48))
#define MPMCDynamicRFC	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x4C))
#define MPMCDynamicXSR	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x50))
#define MPMCDynamicRRD	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x54))
#define MPMCDynamicMRD	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x58))
#define MPMCDynamicCDLR	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x5C))

#define MPMCDynamicConfig0	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x100))
#define MPMCDynamicRasCas0	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x104))

#define MPMCAHBControl0	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x400))
#define MPMCAHBStatus0	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x404))
#define MPMCAHBTimeout0	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x408))

#define MPMCAHBControl2	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x440))
#define MPMCAHBStatus2	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x444))
#define MPMCAHBTimeout2	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x448))

#define MPMCAHBControl3	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x460))
#define MPMCAHBStatus3	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x464))
#define MPMCAHBTimeout3	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x468))

#define MPMCAHBControl4	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x480))
#define MPMCAHBStatus4	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x484))
#define MPMCAHBTimeout4	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x488))

#define DDR_BASE_ADDR	0x40004000
#define DDR_LAP_NOM	(*(volatile unsigned long *)(DDR_BASE_ADDR + 0x6C))
#define DDR_LAP_COUNT	(*(volatile unsigned long *)(DDR_BASE_ADDR + 0x70))
#define DDR_CAL_DELAY	(*(volatile unsigned long *)(DDR_BASE_ADDR + 0x74))
#define RINGOSC_CTRL	(*(volatile unsigned long *)(DDR_BASE_ADDR + 0x88))


/* Interrupt Controller */
#define MIC_BASE_ADDR	0x40008000
#define MIC_ER		(*(volatile unsigned long *)(MIC_BASE_ADDR + 0x00))
#define MIC_RSR		(*(volatile unsigned long *)(MIC_BASE_ADDR + 0x04))
#define MIC_SR		(*(volatile unsigned long *)(MIC_BASE_ADDR + 0x08))
#define MIC_APR		(*(volatile unsigned long *)(MIC_BASE_ADDR + 0x0C))	
#define MIC_ATR		(*(volatile unsigned long *)(MIC_BASE_ADDR + 0x10))
#define MIC_ITR		(*(volatile unsigned long *)(MIC_BASE_ADDR + 0x14))

#define SIC1_BASE_ADDR	0x4000C000
#define SIC1_ER		(*(volatile unsigned long *)(SIC1_BASE_ADDR + 0x00))
#define SIC1_RSR	(*(volatile unsigned long *)(SIC1_BASE_ADDR + 0x04))
#define SIC1_SR		(*(volatile unsigned long *)(SIC1_BASE_ADDR + 0x08))
#define SIC1_APR	(*(volatile unsigned long *)(SIC1_BASE_ADDR + 0x0C))
#define SIC1_ATR	(*(volatile unsigned long *)(SIC1_BASE_ADDR + 0x10))
#define SIC1_ITR	(*(volatile unsigned long *)(SIC1_BASE_ADDR + 0x14))

#define SIC2_BASE_ADDR	0x40010000
#define SIC2_ER		(*(volatile unsigned long *)(SIC2_BASE_ADDR + 0x00))
#define SIC2_RSR	(*(volatile unsigned long *)(SIC2_BASE_ADDR + 0x04))
#define SIC2_SR		(*(volatile unsigned long *)(SIC2_BASE_ADDR + 0x08))
#define SIC2_APR	(*(volatile unsigned long *)(SIC2_BASE_ADDR + 0x0C))
#define SIC2_ATR	(*(volatile unsigned long *)(SIC2_BASE_ADDR + 0x10))
#define SIC2_ITR	(*(volatile unsigned long *)(SIC2_BASE_ADDR + 0x14))

#define SWI_BASE_ADDR	0x40004000
#define SW_INT		(*(volatile unsigned long *)(SWI_BASE_ADDR + 0xA8))


/* Multiple level NAND Flash */
#define MLC_BASE_ADDR	0x200B8000
#define MLC_CMD		(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x00))
#define MLC_ADDR	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x04))
#define MLC_ECC_ENC_REG	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x08))
#define MLC_ECC_DEC_REG	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x0C))
#define MLC_ECC_AUTO_ENC_REG	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x10))
#define MLC_ECC_AUTO_DEC_REG	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x14))
#define MLC_RPR		(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x18))
#define MLC_WPR		(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x1C))
#define MLC_RUBP	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x20))
#define MLC_ROBP	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x24))
#define MLC_SW_WP_ADD_LOW	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x28))
#define MLC_SW_WP_ADD_HIG	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x2C))
#define MLC_ICR		(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x30))
#define MLC_TIME_REG	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x34))
#define MLC_IRQ_MR	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x38))
#define MLC_IRQ_SR	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x3C))
#define MLC_LOCK_PR	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x44))
#define MLC_ISR		(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x48))
#define MLC_CEH		(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x4C))


/* Single level NAND Flash */
#define SLC_BASE_ADDR	0x20020000
#define SLC_DATA	(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x00))
#define SLC_ADDR	(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x04))
#define SLC_CMD		(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x08))
#define SLC_STOP	(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x0C))
#define SLC_CTRL	(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x10))
#define SLC_CFG		(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x14))
#define SLC_STAT	(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x18))
#define SLC_INT_STAT	(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x1C))
#define SLC_IEN		(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x20))
#define SLC_ISR		(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x24))
#define SLC_ICR		(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x28))
#define SLC_TAC		(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x2C))
#define SLC_TC		(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x30))
#define SLC_ECC		(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x34))
#define SLC_DMA_DATA	(*(volatile unsigned long *)(SLC_BASE_ADDR + 0x38))


/* GPIOs */
#define GPIO_BASE_ADDR	0x40028000
#define PIO_INP_STATE	(*(volatile unsigned long *)(GPIO_BASE_ADDR + 0x00))
#define PIO_OUTP_SET	(*(volatile unsigned long *)(GPIO_BASE_ADDR + 0x04))
#define PIO_OUTP_CLR	(*(volatile unsigned long *)(GPIO_BASE_ADDR + 0x08))
#define PIO_OUTP_STATE	(*(volatile unsigned long *)(GPIO_BASE_ADDR + 0x0C))
#define PIO_DIR_SET	(*(volatile unsigned long *)(GPIO_BASE_ADDR + 0x10))
#define PIO_DIR_CLR	(*(volatile unsigned long *)(GPIO_BASE_ADDR + 0x14))
#define PIO_DIR_STATE	(*(volatile unsigned long *)(GPIO_BASE_ADDR + 0x18))
#define PIO_SDINP_STATE	(*(volatile unsigned long *)(GPIO_BASE_ADDR + 0x1C))
#define PIO_SDOUTP_SET	(*(volatile unsigned long *)(GPIO_BASE_ADDR + 0x20))
#define PIO_SDOUTP_CLR	(*(volatile unsigned long *)(GPIO_BASE_ADDR + 0x24))
#define PIO_MUX_SET	(*(volatile unsigned long *)(GPIO_BASE_ADDR + 0x28))
#define PIO_MUX_CLR	(*(volatile unsigned long *)(GPIO_BASE_ADDR + 0x2C))
#define PIO_MUX_STATE	(*(volatile unsigned long *)(GPIO_BASE_ADDR + 0x30))


/* USB Device */
#define USB_BASE_ADDR		0x31020200

/* Device Interrupt Registers */
#define DEV_INT_STAT        (*(volatile unsigned long *)(USB_BASE_ADDR + 0x00))
#define DEV_INT_EN          (*(volatile unsigned long *)(USB_BASE_ADDR + 0x04))
#define DEV_INT_CLR         (*(volatile unsigned long *)(USB_BASE_ADDR + 0x08))
#define DEV_INT_SET         (*(volatile unsigned long *)(USB_BASE_ADDR + 0x0C))
#define DEV_INT_PRIO        (*(volatile unsigned long *)(USB_BASE_ADDR + 0x2C))

/* Endpoint Interrupt Registers */
#define EP_INT_STAT         (*(volatile unsigned long *)(USB_BASE_ADDR + 0x30))
#define EP_INT_EN           (*(volatile unsigned long *)(USB_BASE_ADDR + 0x34))
#define EP_INT_CLR          (*(volatile unsigned long *)(USB_BASE_ADDR + 0x38))
#define EP_INT_SET          (*(volatile unsigned long *)(USB_BASE_ADDR + 0x3C))
#define EP_INT_PRIO         (*(volatile unsigned long *)(USB_BASE_ADDR + 0x40))

/* Endpoint Realization Registers */
#define REALIZE_EP          (*(volatile unsigned long *)(USB_BASE_ADDR + 0x44))
#define EP_INDEX            (*(volatile unsigned long *)(USB_BASE_ADDR + 0x48))
#define MAXPACKET_SIZE      (*(volatile unsigned long *)(USB_BASE_ADDR + 0x4C))

/* Command Reagisters */
#define CMD_CODE            (*(volatile unsigned long *)(USB_BASE_ADDR + 0x10))
#define CMD_DATA            (*(volatile unsigned long *)(USB_BASE_ADDR + 0x14))

/* Data Transfer Registers */
#define RX_DATA             (*(volatile unsigned long *)(USB_BASE_ADDR + 0x18))
#define TX_DATA             (*(volatile unsigned long *)(USB_BASE_ADDR + 0x1C))
#define RX_PLENGTH          (*(volatile unsigned long *)(USB_BASE_ADDR + 0x20))
#define TX_PLENGTH          (*(volatile unsigned long *)(USB_BASE_ADDR + 0x24))
#define USB_CTRL            (*(volatile unsigned long *)(USB_BASE_ADDR + 0x28))

/* System Register */
#define DC_REVISION         (*(volatile unsigned long *)(USB_BASE_ADDR + 0x7C))

/* DMA Registers */
#define DMA_REQ_STAT        (*(volatile unsigned long *)(USB_BASE_ADDR + 0x50))
#define DMA_REQ_CLR         (*(volatile unsigned long *)(USB_BASE_ADDR + 0x54))
#define DMA_REQ_SET         (*(volatile unsigned long *)(USB_BASE_ADDR + 0x58))
#define UDCA_HEAD           (*(volatile unsigned long *)(USB_BASE_ADDR + 0x80))
#define EP_DMA_STAT         (*(volatile unsigned long *)(USB_BASE_ADDR + 0x84))
#define EP_DMA_EN           (*(volatile unsigned long *)(USB_BASE_ADDR + 0x88))
#define EP_DMA_DIS          (*(volatile unsigned long *)(USB_BASE_ADDR + 0x8C))
#define DMA_INT_STAT        (*(volatile unsigned long *)(USB_BASE_ADDR + 0x90))
#define DMA_INT_EN          (*(volatile unsigned long *)(USB_BASE_ADDR + 0x94))
#define EOT_INT_STAT        (*(volatile unsigned long *)(USB_BASE_ADDR + 0xA0))
#define EOT_INT_CLR         (*(volatile unsigned long *)(USB_BASE_ADDR + 0xA4))
#define EOT_INT_SET         (*(volatile unsigned long *)(USB_BASE_ADDR + 0xA8))
#define NDD_REQ_INT_STAT    (*(volatile unsigned long *)(USB_BASE_ADDR + 0xAC))
#define NDD_REQ_INT_CLR     (*(volatile unsigned long *)(USB_BASE_ADDR + 0xB0))
#define NDD_REQ_INT_SET     (*(volatile unsigned long *)(USB_BASE_ADDR + 0xB4))
#define SYS_ERR_INT_STAT    (*(volatile unsigned long *)(USB_BASE_ADDR + 0xB8))
#define SYS_ERR_INT_CLR     (*(volatile unsigned long *)(USB_BASE_ADDR + 0xBC))
#define SYS_ERR_INT_SET     (*(volatile unsigned long *)(USB_BASE_ADDR + 0xC0))
#define MODULE_ID           (*(volatile unsigned long *)(USB_BASE_ADDR + 0xFC))


/* USB Host Controller */
#define USBH_BASE_ADDR	0x31020000
#define HcRevision	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x00))
#define HcControl	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x04))
#define HcCommandStatus	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x08))
#define HcInterruptStatus	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x0C))
#define HcInterruptEnable	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x10))
#define HcInterruptDisable	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x14))
#define HcHCCA		(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x18))
#define HcPeriodCurrentED	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x1C))
#define HcControlHeadED	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x20))
#define HcControlCurrentED	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x24))
#define HcBulkHeadED	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x28))
#define HcBulkCurrentED	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x2C))
#define HcDoneHead	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x30))
#define HcFmInterval	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x34))
#define HcFmRemaining	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x38))
#define HcFmNumber	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x3C))
#define HcPeriodicStart	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x40))
#define HcLSThreshold	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x44))
#define HcRhDescriptorA	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x48))
#define HcRhDescriptorB	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x4C))
#define HcRhStatus	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x50))
#define HcRhPortStatus1	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x54))
#define HcRhPortStatus2	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x58))
#define HcModuleID	(*(volatile unsigned long *)(USBH_BASE_ADDR + 0x5C))


/* USB OTG */
#define USB_OTG_BASE_ADDR	0x31020000
#define OTG_int_status	(*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0x100))
#define OTG_int_enable	(*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0x104))
#define OTG_int_set	(*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0x108))
#define OTG_int_clear	(*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0x10C))
#define OTG_status_ctrl	(*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0x110))
#define OTG_timer	(*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0x114))

/* OTG clock control registers */
#define OTG_clock_control	(*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0xFF4))
#define OTG_clock_status	(*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0xFF8))
#define OTG_module_id		(*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0xFFC))

/* USB OTG I2C register */
#define OTG_I2C_BASE_ADDR	0x31020300
#define I2C_RX		(*(volatile unsigned long *)(OTG_I2C_BASE_ADDR + 0x00))
#define I2C_TX		(*(volatile unsigned long *)(OTG_I2C_BASE_ADDR + 0x00))
#define I2C_STS		(*(volatile unsigned long *)(OTG_I2C_BASE_ADDR + 0x04))
#define I2C_CTRL	(*(volatile unsigned long *)(OTG_I2C_BASE_ADDR + 0x08))
#define I2C_CLKH	(*(volatile unsigned long *)(OTG_I2C_BASE_ADDR + 0x0C))
#define I2C_CLKL	(*(volatile unsigned long *)(OTG_I2C_BASE_ADDR + 0x10))


/* Universal Asynchronous Receiver Transmitter (Standard UART3,4,5,6) */
#define UART3_BASE_ADDR		0x40080000
#define U3RBR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x00))
#define U3THR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x00))
#define U3DLL          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x00))
#define U3DLM          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x04))
#define U3IER          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x04))
#define U3IIR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x08))
#define U3FCR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x08))
#define U3LCR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x0C))
#define U3LSR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x14))
#define U3RXLEV        (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x1C))

#define UART4_BASE_ADDR		0x40088000
#define U4RBR          (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x00))
#define U4THR          (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x00))
#define U4DLL          (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x00))
#define U4DLM          (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x04))
#define U4IER          (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x04))
#define U4IIR          (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x08))
#define U4FCR          (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x08))
#define U4LCR          (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x0C))
#define U4LSR          (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x14))
#define U4RXLEV        (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x1C))

#define UART5_BASE_ADDR		0x40090000
#define U5RBR          (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x00))
#define U5THR          (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x00))
#define U5DLL          (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x00))
#define U5DLM          (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x04))
#define U5IER          (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x04))
#define U5IIR          (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x08))
#define U5FCR          (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x08))
#define U5LCR          (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x0C))
#define U5LSR          (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x14))
#define U5RXLEV        (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x1C))

#define UART6_BASE_ADDR		0x40098000
#define U6RBR          (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x00))
#define U6THR          (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x00))
#define U6DLL          (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x00))
#define U6DLM          (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x04))
#define U6IER          (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x04))
#define U6IIR          (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x08))
#define U6FCR          (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x08))
#define U6LCR          (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x0C))
#define U6LSR          (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x14))
#define U6RXLEV        (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x1C))

#define UART_CLK_BASE_ADDR	0x40004000
#define U3CLK          (*(volatile unsigned long *)(UART_CLK_BASE_ADDR + 0xD0))
#define U4CLK          (*(volatile unsigned long *)(UART_CLK_BASE_ADDR + 0xD4))
#define U5CLK          (*(volatile unsigned long *)(UART_CLK_BASE_ADDR + 0xD8))
#define U6CLK          (*(volatile unsigned long *)(UART_CLK_BASE_ADDR + 0xDC))
#define IRDACLK        (*(volatile unsigned long *)(UART_CLK_BASE_ADDR + 0xE0))

#define UART_CTRL_BASE_ADDR	0x40054000
#define UART_CTRL      (*(volatile unsigned long *)(UART_CTRL_BASE_ADDR + 0x00))
#define UART_CLKMODE   (*(volatile unsigned long *)(UART_CTRL_BASE_ADDR + 0x04))
#define UART_LOOP      (*(volatile unsigned long *)(UART_CTRL_BASE_ADDR + 0x08))


/* High-speed Universal Asynchronous Receiver Transmitter (UART1,2,7) */
#define HS_UART1_BASE_ADDR		0x40014000
#define HSU1_RX        (*(volatile unsigned long *)(HS_UART1_BASE_ADDR + 0x00))
#define HSU1_TX        (*(volatile unsigned long *)(HS_UART1_BASE_ADDR + 0x00))
#define HSU1_LEVEL     (*(volatile unsigned long *)(HS_UART1_BASE_ADDR + 0x04))
#define HSU1_IIR       (*(volatile unsigned long *)(HS_UART1_BASE_ADDR + 0x08))
#define HSU1_CTRL      (*(volatile unsigned long *)(HS_UART1_BASE_ADDR + 0x0C))
#define HSU1_RATE      (*(volatile unsigned long *)(HS_UART1_BASE_ADDR + 0x10))

#define HS_UART2_BASE_ADDR		0x40018000
#define HSU2_RX        (*(volatile unsigned long *)(HS_UART2_BASE_ADDR + 0x00))
#define HSU2_TX        (*(volatile unsigned long *)(HS_UART2_BASE_ADDR + 0x00))
#define HSU2_LEVEL     (*(volatile unsigned long *)(HS_UART2_BASE_ADDR + 0x04))
#define HSU2_IIR       (*(volatile unsigned long *)(HS_UART2_BASE_ADDR + 0x08))
#define HSU2_CTRL      (*(volatile unsigned long *)(HS_UART2_BASE_ADDR + 0x0C))
#define HSU2_RATE      (*(volatile unsigned long *)(HS_UART2_BASE_ADDR + 0x10))

#define HS_UART7_BASE_ADDR		0x4001C000
#define HSU7_RX        (*(volatile unsigned long *)(HS_UART7_BASE_ADDR + 0x00))
#define HSU7_TX        (*(volatile unsigned long *)(HS_UART7_BASE_ADDR + 0x00))
#define HSU7_LEVEL     (*(volatile unsigned long *)(HS_UART7_BASE_ADDR + 0x04))
#define HSU7_IIR       (*(volatile unsigned long *)(HS_UART7_BASE_ADDR + 0x08))
#define HSU7_CTRL      (*(volatile unsigned long *)(HS_UART7_BASE_ADDR + 0x0C))
#define HSU7_RATE      (*(volatile unsigned long *)(HS_UART7_BASE_ADDR + 0x10))


/* SPI Interface */
#define SPI1_BASE_ADDR	0x20088000
#define SPI1_GLOBAL	(*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x000))
#define SPI1_CON	(*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x004))
#define SPI1_FRM	(*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x008))
#define SPI1_IER	(*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x00C))
#define SPI1_STAT	(*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x010))
#define SPI1_DAT	(*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x014))
#define SPI1_TIM_CTRL	(*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x400))
#define SPI1_TIM_COUNT	(*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x404))
#define SPI1_TIM_STAT	(*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x408))

#define SPI2_BASE_ADDR	0x20090000
#define SPI2_GLOBAL	(*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x000))
#define SPI2_CON	(*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x004))
#define SPI2_FRM	(*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x008))
#define SPI2_IER	(*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x00C))
#define SPI2_STAT	(*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x010))
#define SPI2_DAT	(*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x014))
#define SPI2_TIM_CTRL	(*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x400))
#define SPI2_TIM_COUNT	(*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x404))
#define SPI2_TIM_STAT	(*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x408))


/* SD Card Interface */
#define SD_BASE_ADDR	0x20098000
#define SD_POWER	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x00))
#define SD_CLOCK	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x04))
#define SD_ARGUMENT	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x08))
#define SD_COMMAND	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x0C))
#define SD_RESPCMD	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x10))
#define SD_RESPONSE0	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x14))
#define SD_RESPONSE1	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x18))
#define SD_RESPONSE2	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x1C))
#define SD_RESPONSE3	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x20))
#define SD_DATATIMER	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x24))
#define SD_DATALENGTH	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x28))
#define SD_DATACTRL	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x2C))
#define SD_DATACNT	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x30))
#define SD_STATUS	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x34))
#define SD_CLEAR	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x38))
#define SD_MASK0	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x3C))
#define SD_MASK1	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x40))
#define SD_FIFOCNT	(*(volatile unsigned long *)(SD_BASE_ADDR + 0x48))
/* Note: for FIFO register, the addr. is 0x20098080 through 0x200980BC */
#define SD_FIFO		(*(volatile unsigned long *)(SD_BASE_ADDR + 0x80))


/* I2C Interface 0 */
#define I2C0_BASE_ADDR		0x400A0000
#define I2C0_RX		(*(volatile unsigned long *)(I2C0_BASE_ADDR + 0x00))
#define I2C0_TX		(*(volatile unsigned long *)(I2C0_BASE_ADDR + 0x00))
#define I2C0_STS	(*(volatile unsigned long *)(I2C0_BASE_ADDR + 0x04))
#define I2C0_CTRL	(*(volatile unsigned long *)(I2C0_BASE_ADDR + 0x08))
#define I2C0_CLKH	(*(volatile unsigned long *)(I2C0_BASE_ADDR + 0x0C))
#define I2C0_CLKL	(*(volatile unsigned long *)(I2C0_BASE_ADDR + 0x10))

/* I2C Interface 1 */
#define I2C1_BASE_ADDR		0x400A8000
#define I2C1_RX		(*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x00))
#define I2C1_TX		(*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x00))
#define I2C1_STS	(*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x04))
#define I2C1_CTRL	(*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x08))
#define I2C1_CLKH	(*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x0C))
#define I2C1_CLKL	(*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x10))


/* Keyboard Scan controller interface */
#define KS_BASE_ADDR		0x40050000
#define KS_DEB		(*(volatile unsigned long *)(KS_BASE_ADDR + 0x00))
#define KS_STATE_COND	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x04))
#define KS_IRQ		(*(volatile unsigned long *)(KS_BASE_ADDR + 0x08))
#define KS_SCAN_CTL	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x0C))
#define KS_FAST_TST	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x10))
#define KS_MATRIX_DIM	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x14))
#define KS_DATA0	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x40))
#define KS_DATA1	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x44))
#define KS_DATA2	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x48))
#define KS_DATA3	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x4C))
#define KS_DATA4	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x50))
#define KS_DATA5	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x54))
#define KS_DATA6	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x58))
#define KS_DATA7	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x5C))


/* High-speed Timer  */
#define HSTIM_BASE_ADDR		0x40038000
#define HSTIM_INT	(*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x00))
#define HSTIM_CTRL	(*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x04))
#define HSTIM_COUNTER	(*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x08))
#define HSTIM_PMATCH	(*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x0C))
#define HSTIM_PCOUNT	(*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x10))
#define HSTIM_MCTRL	(*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x14))
#define HSTIM_MATCH0	(*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x18))
#define HSTIM_MATCH1	(*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x1C))
#define HSTIM_MATCH2	(*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x20))
#define HSTIM_CCR	(*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x28))
#define HSTIM_CR0	(*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x2C))
#define HSTIM_CR1	(*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x30))


/* Millisecond Timer */
#define MSTIM_BASE_ADDR		0x40034000
#define MSTIM_INT	(*(volatile unsigned long *)(MSTIM_BASE_ADDR + 0x00))
#define MSTIM_CTRL	(*(volatile unsigned long *)(MSTIM_BASE_ADDR + 0x04))
#define MSTIM_COUNTER	(*(volatile unsigned long *)(MSTIM_BASE_ADDR + 0x08))
#define MSTIM_MCTRL	(*(volatile unsigned long *)(MSTIM_BASE_ADDR + 0x14))
#define MSTIM_MATCH0	(*(volatile unsigned long *)(MSTIM_BASE_ADDR + 0x18))
#define MSTIM_MATCH1	(*(volatile unsigned long *)(MSTIM_BASE_ADDR + 0x1C))


/* Real Time Clock */
#define RTC_BASE_ADDR		0x40024000
#define RTC_UCOUNT	(*(volatile unsigned long *)(RTC_BASE_ADDR + 0x00))
#define RTC_DCOUNT	(*(volatile unsigned long *)(RTC_BASE_ADDR + 0x04))
#define RTC_MATCH0	(*(volatile unsigned long *)(RTC_BASE_ADDR + 0x08))
#define RTC_MATCH1	(*(volatile unsigned long *)(RTC_BASE_ADDR + 0x0C))
#define RTC_CTRL	(*(volatile unsigned long *)(RTC_BASE_ADDR + 0x10))
#define RTC_INTSTAT	(*(volatile unsigned long *)(RTC_BASE_ADDR + 0x14))
#define RTC_KEY		(*(volatile unsigned long *)(RTC_BASE_ADDR + 0x18))
#define RTC_SRAM	(*(volatile unsigned long *)(RTC_BASE_ADDR + 0x80))


/* Watchdog timer */
#define WDTIM_BASE_ADDR		0x4003C000
#define WDTIM_INT	(*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x00))
#define WDTIM_CTRL	(*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x04))
#define WDTIM_COUNTER	(*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x08))
#define WDTIM_MCTRL	(*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x0C))
#define WDTIM_MATCH0	(*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x10))
#define WDTIM_EMR	(*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x14))
#define WDTIM_PULSE	(*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x18))
#define WDTIM_RES	(*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x1C))


/* A/D Converter */
#define ADC_BASE_ADDR		0x40048000
#define ADSTAT		(*(volatile unsigned long *)(AD0_BASE_ADDR + 0x00))
#define ADSEL		(*(volatile unsigned long *)(AD0_BASE_ADDR + 0x04))
#define ADCON		(*(volatile unsigned long *)(AD1_BASE_ADDR + 0x08))
#define ADDAT		(*(volatile unsigned long *)(AD1_BASE_ADDR + 0x48))


/* DMA */
#define DMAC_BASE_ADDR		0x31000000
#define DMAC_INT_STAT	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x000))
#define DMAC_INT_TCSTAT	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x004))
#define DMAC_INT_TCCLEAR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x008))
#define DMAC_INT_ERRSTAT	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x00C))
#define DMAC_INT_ERRCLR		(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x010))
#define DMAC_RAW_INT_TCSTAT	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x014))
#define DMAC_RAW_INT_ERRSTAT	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x018))
#define DMAC_ENBLD_CHNS	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x01C))
#define DMAC_SOFT_BREQ	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x020))
#define DMAC_SOFT_SREQ	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x024))
#define DMAC_SOFT_LBREQ	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x028))
#define DMAC_SOFT_LSREQ	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x02C))
#define DMAC_CONFIG	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x030))
#define DMAC_SYNC	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x034))

#define DMAC_C0_SRC_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x100))
#define DMAC_C0_DEST_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x104))
#define DMAC_C0_LLI	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x108))
#define DMAC_C0_CONTROL	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x10C))
#define DMAC_C0_CONFIG	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x110))

#define DMAC_C1_SRC_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x120))
#define DMAC_C1_DEST_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x124))
#define DMAC_C1_LLI	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x128))
#define DMAC_C1_CONTROL	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x12C))
#define DMAC_C1_CONFIG	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x130))

#define DMAC_C2_SRC_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x140))
#define DMAC_C2_DEST_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x144))
#define DMAC_C2_LLI	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x148))
#define DMAC_C2_CONTROL	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x14C))
#define DMAC_C2_CONFIG	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x150))

#define DMAC_C3_SRC_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x160))
#define DMAC_C3_DEST_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x164))
#define DMAC_C3_LLI	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x168))
#define DMAC_C3_CONTROL	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x16C))
#define DMAC_C3_CONFIG	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x170))

#define DMAC_C4_SRC_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x180))
#define DMAC_C4_DEST_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x184))
#define DMAC_C4_LLI	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x188))
#define DMAC_C4_CONTROL	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x18C))
#define DMAC_C4_CONFIG	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x190))

#define DMAC_C5_SRC_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1A0))
#define DMAC_C5_DEST_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1A4))
#define DMAC_C5_LLI	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1A8))
#define DMAC_C5_CONTROL	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1AC))
#define DMAC_C5_CONFIG	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1B0))

#define DMAC_C6_SRC_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1C0))
#define DMAC_C6_DEST_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1C4))
#define DMAC_C6_LLI	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1C8))
#define DMAC_C6_CONTROL	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1CC))
#define DMAC_C6_CONFIG	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1D0))

#define DMAC_C7_SRC_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1E0))
#define DMAC_C7_DEST_ADDR	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1E4))
#define DMAC_C7_LLI	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1E8))
#define DMAC_C7_CONTROL	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1EC))
#define DMAC_C7_CONFIG	(*(volatile unsigned long *)(DMAC_BASE_ADDR + 0x1F0))


#endif  // __LPC318x_H

