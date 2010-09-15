ARCHI=avr

ap.ARCHDIR = $(ARCHI)
ap.ARCH = atmega128
ap.LOW_FUSE  = a0
ap.HIGH_FUSE = 99
ap.EXT_FUSE  = ff
ap.LOCK_FUSE = ff
ap.CFLAGS += -DAP

fbw.ARCHDIR = $(ARCHI)
fbw.ARCH = atmega8
fbw.LOW_FUSE  = 2e
fbw.HIGH_FUSE = cb
fbw.EXT_FUSE  = ff
fbw.LOCK_FUSE = ff
fbw.CFLAGS += -DFBW

tunnel.ARCHDIR = $(ARCHI)
tunnel.ARCH = atmega128
tunnel.srcs += $(SRC_ARCH)/uart_tunnel.c
