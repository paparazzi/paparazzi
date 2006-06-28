ARCHI=avr

ap.ARCHDIR = $(ARCHI)
ap.ARCH = atmega128
ap.TARGET = autopilot
ap.TARGETDIR = autopilot

# default values ( 1MHz internal )
#ap.LOW_FUSE  = E1
#ap.HIGH_FUSE = 99
#ap.EXT_FUSE  = ff
#ap.LOCK_FUSE = ff

# 8MHz internal
ap.LOW_FUSE  = E4
ap.HIGH_FUSE = 99
ap.EXT_FUSE  = ff
ap.LOCK_FUSE = ff

