LIBNAME	= libusbstack

# Package definitions
PKG_NAME	= target
DATE		= $$(date +%Y%m%d)

# Tool definitions
# try to find the paparazzi multilib toolchain
TOOLCHAIN_DIR=$(shell find -L /opt/paparazzi/arm-multilib ~/sat -maxdepth 1 -type d -name arm-none-eabi 2>/dev/null | head -n 1 | xargs -r dirname )

#
# found the new paparazzi toolchain, use it
#
ifneq ($(TOOLCHAIN_DIR),)
GCC_BIN_DIR=$(TOOLCHAIN_DIR)/bin
GCC_LIB_DIR=$(TOOLCHAIN_DIR)/arm-none-eabi/lib
GCC_BIN_PREFIX=$(GCC_BIN_DIR)/arm-none-eabi
CC      = $(GCC_BIN_PREFIX)-gcc
LD      = $(GCC_BIN_PREFIX)-ld -v
AR      = $(GCC_BIN_PREFIX)-ar
AS      = $(GCC_BIN_PREFIX)-as
CP      = $(GCC_BIN_PREFIX)-objcopy
OD      = $(GCC_BIN_PREFIX)-objdump

#
# If we can't find the toolchain (in /opt/paparazzi/arm-multilib or ~/sat) then try picking up the compilers from the path
#
else

# see if we can find the new arm-none-eabi, otherwise use the older arm-elf
HAVE_ARM_NONE_EABI_GCC := $(shell which arm-none-eabi-gcc)
ifeq ($(strip $(HAVE_ARM_NONE_EABI_GCC)),)
GCC_PREFIX = arm-elf
else
GCC_PREFIX = arm-none-eabi
GCC_LIB_DIR=$(shell dirname `which arm-none-eabi-gcc`)/../arm-none-eabi/lib
endif

CC      = $(shell which $(GCC_PREFIX)-gcc)
LD      = $(shell which $(GCC_PREFIX)-ld) -v
AR      = $(shell which $(GCC_PREFIX)-ar)
AS      = $(shell which $(GCC_PREFIX)-as)
CP      = $(shell which $(GCC_PREFIX)-objcopy)
OD      = $(shell which $(GCC_PREFIX)-objdump)
endif

# Define some other programs and commands.
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
