ARCHI=avr

ap.ARCHDIR = $(ARCHI)
ap.ARCH = atmega128
ap.TARGET = autopilot
ap.TARGETDIR = autopilot

# 16MHz resonator
#ap.LOW_FUSE  = CF
#ap.HIGH_FUSE = 89
#ap.EXT_FUSE  = ff
#ap.LOCK_FUSE = ff


# default values
#ap.LOW_FUSE  = E1
#ap.HIGH_FUSE = 99
#ap.EXT_FUSE  = ff
#ap.LOCK_FUSE = ff


ap.CFLAGS += -DFBW -DCONFIG=\"gorrazoptere_091.h\"  
ap.srcs = $(SRC_ARCH)/adc_hw.c sys_time.c main_fbw.c main.c
ap.CFLAGS += -DANTON_QUAD -DLED
# pid.c estimator.c if_calib.c nav.c main_ap.c mainloop.c main.c
