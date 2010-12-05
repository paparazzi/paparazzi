ARCH=avr

ap.ARCHDIR = $(ARCH)
ap.MCU = atmega128
ap.LOW_FUSE  = a0
ap.HIGH_FUSE = 99
ap.EXT_FUSE  = ff
ap.LOCK_FUSE = ff
ap.CFLAGS += -DAP

fbw.ARCHDIR = $(ARCH)
fbw.MCU = atmega8
fbw.LOW_FUSE  = 2e
fbw.HIGH_FUSE = cb
fbw.EXT_FUSE  = ff
fbw.LOCK_FUSE = ff
fbw.CFLAGS += -DFBW

tunnel.ARCHDIR = $(ARCH)
tunnel.MCU = atmega128
tunnel.srcs += $(SRC_ARCH)/uart_tunnel.c
