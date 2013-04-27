# Hey Emacs, this is a -*- makefile -*-
#
#   Copyright (C) 2004 Pascal Brisset, Antoine Drouin
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

# The default is to produce a quiet echo of compilation commands
# Launch with "make Q=''" to get full echo
Q=@

LIBNAME	= libusbstack

# Package definitions
PKG_NAME	= target
DATE		= $$(date +%Y%m%d)

#
# find compiler toolchain
#
include $(PAPARAZZI_SRC)/conf/Makefile.arm-embedded-toolchain

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
	@echo CLEAN $(LIBNAME)
	$(Q)$(RM) -f $(LIBNAME).a $(LIBOBJS) .depend

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
	$(Q)$(CC) $(CFLAGS) -MM $^ > .depend || rm -f .depend

# phony targets
.PHONY: all clean lib dist depend

-include .depend
