##
## Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
##
## This file is part of Paparazzi.
##
## Paparazzi is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2, or (at your option)
## any later version.
##
## Paparazzi is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with paparazzi; see the file COPYING.  If not, see
## <http://www.gnu.org/licenses/>.
##

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
endif

VPATH = lib/arm


ifeq ($(OS),Windows_NT)
  $(warning Warning: OpenGL ES dummy compilation on Windows not supported)
else
  UNAME_S := $(shell uname -s)
  ifeq ($(UNAME_S),Linux)
    PREFIX    ?= /opt/arm-2012.03/bin/arm-none-linux-gnueabi
  else
    ifeq ($(UNAME_S),Darwin)
      PREFIX    ?= /usr/local/carlson-minot/crosscompilers/bin/arm-none-linux-gnueabi
    endif
  endif
endif

CC        = $(PREFIX)-gcc
CFLAGS    = -Wall
LDFLAGS   =


all: GLESv2.so EGL.so GLESv1_CM.so

%.so: %.o
	@printf "  LIB     $(<F)\n"
	$(Q)$(CC) -shared $(LDLAGS) $< -o $(VPATH)/lib$@

%.o: %.c
	@printf "  CC      $(<F)\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

clean:
	$(Q)rm -f *.o *.d *.so

.PHONY: clean

