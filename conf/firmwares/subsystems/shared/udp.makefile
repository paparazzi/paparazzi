# Hey Emacs, this is a -*- makefile -*-

ifndef UDP_INCLUDED

UDP_INCLUDED = 1

#generic spi master driver
UDP_CFLAGS = -DUSE_UDP
UDP_SRCS = mcu_periph/udp.c $(SRC_ARCH)/mcu_periph/udp_arch.c

$(TARGET).CFLAGS += $(UDP_CFLAGS)
$(TARGET).srcs += $(UDP_SRCS)

endif
