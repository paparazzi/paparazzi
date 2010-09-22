LIBNAME	= libusbstack

# Package definitions
PKG_NAME	= target
DATE		= $$(date +%Y%m%d)

# Tool definitions
HAVE_ARM_NONE_EABI_GCC := $(wildcard /usr/bin/arm-none-eabi-gcc)

ifeq ($(strip $(HAVE_ARM_NONE_EABI_GCC)),)
CC      = arm-elf-gcc
LD      = arm-elf-ld -v
AR      = arm-elf-ar
AS      = arm-elf-as
CP      = arm-elf-objcopy
OD	= arm-elf-objdump
else
CC=arm-none-eabi-gcc
LD=arm-none-eabi-ld -v
AR=arm-none-eabi-ar
AS=arm-none-eabi-ar
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
endif

RM		= rm
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
