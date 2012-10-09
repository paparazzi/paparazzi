LIBNAME	= libusbstack

# Package definitions
PKG_NAME	= target
DATE		= $$(date +%Y%m%d)

#
# find compiler toolchain
#
include $(PAPARAZZI_SRC)/conf/Makefile.arm-common

#
# if the new arm-none-eabi multilib compiler was not found try the old arm-elf one
#
ifeq ($(CC),)
CC   = $(shell which arm-elf-gcc)
LD   = $(shell which arm-elf-gcc)
AR   = $(shell which arm-elf-ar)
CP   = $(shell which arm-elf-objcopy)
DMP  = $(shell which arm-elf-objdump)
NM   = $(shell which arm-elf-nm)
SIZE = $(shell which arm-elf-size)
endif


# Define some other programs and commands.
TAR		= tar

CFLAGS  = -I./ -I../ -c -W -Wall -Os -g -mcpu=arm7tdmi
# -mthumb -mthumb-interwork
ARFLAGS = -rcs

LIBSRCS = usbhw_lpc.c usbcontrol.c usbstdreq.c usbinit.c
LIBOBJS = $(LIBSRCS:.c=.o)

all: depend lib

clean:
	$(RM) -f $(LIBNAME).a $(LIBOBJS) .depend

# build lib
lib: $(LIBNAME).a

$(LIBNAME).a: $(LIBOBJS)
	$(AR) $(ARFLAGS) $@ $^

# Builds release tar file
dist: clean
	cd .. && $(TAR) --exclude={CVS,cvs,.svn} -cvzf $(PKG_NAME)-$(DATE).tar.gz $(PKG_NAME)

# recompile if the Makefile changes
$(LIBOBJS): Makefile

# dependency checking
depend: $(LIBSRCS)
	$(CC) $(CFLAGS) -MM $^ > .depend || rm -f .depend

# phony targets
.PHONY: all clean depend

-include .depend
