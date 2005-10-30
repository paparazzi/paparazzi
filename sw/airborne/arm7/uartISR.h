/******************************************************************************
 *
 * $RCSfile$
 * $Revision$
 *
 * This module provides the interface definitions for uartISR.c
 * Copyright 2004, R O SoftWare
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 *****************************************************************************/
#ifndef INC_UART_ISR_H
#define INC_UART_ISR_H

#include "uart.h"

#if UART0_SUPPORT
#if defined(UART0_TX_INT_MODE) || defined(UART0_RX_INT_MODE)
void uart0ISR(void) __attribute__((naked));
#endif // UART0_TX_INT_MODE || UART0_RX_INT_MODE
#endif // UART0_SUPPORT


#if UART1_SUPPORT
#if defined(UART1_TX_INT_MODE) || defined(UART1_RX_INT_MODE)
void uart1ISR(void) __attribute__((naked));
#endif // UART1_TX_INT_MODE || UART1_RX_INT_MODE
#endif // UART1_SUPPORT

#endif
