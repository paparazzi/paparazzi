# Hey Emacs, this is a -*- makefile -*-

ifndef UART_INCLUDED

UART_INCLUDED = 1

UART_SRCS = mcu_periph/uart.c $(SRC_ARCH)/mcu_periph/uart_arch.c
ifeq ($(ARCH), linux)
UART_SRCS += $(SRC_ARCH)/serial_port.c
endif
ifeq ($(TARGET), nps)
UART_CFLAGS += -Iarch/linux
UART_SRCS += arch/linux/serial_port.c
endif
ifeq ($(TARGET), hitl)
UART_CFLAGS += -Iarch/linux
UART_SRCS += arch/linux/serial_port.c
endif
ifeq ($(TARGET), sim)
UART_CFLAGS += -Iarch/linux
UART_SRCS += arch/linux/serial_port.c
endif

$(TARGET).CFLAGS += $(UART_CFLAGS)
$(TARGET).srcs += $(UART_SRCS)

endif
