# Hey Emacs, this is a -*- makefile -*-

ifndef UDP_INCLUDED

UDP_INCLUDED = 1

UDP_CFLAGS = -DUSE_UDP
UDP_SRCS = mcu_periph/udp.c $(SRC_ARCH)/mcu_periph/udp_arch.c
ifeq ($(ARCH), linux)
UDP_SRCS += $(SRC_ARCH)/udp_socket.c
endif
ifeq ($(TARGET), nps)
UDP_CFLAGS += -Iarch/linux
UDP_SRCS += arch/linux/udp_socket.c
endif

$(TARGET).CFLAGS += $(UDP_CFLAGS)
$(TARGET).srcs += $(UDP_SRCS)

endif
