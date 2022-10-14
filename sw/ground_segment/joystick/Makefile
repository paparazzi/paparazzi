# Hey Emacs, this is a -*- makefile -*-
#
#   Copyright (C) 2004 Pascal Brisset, Antoine Drouin
#   Copyright (C) 2012 The Paparazzi Team
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
#

# Quiet compilation
Q=@

include ../../Makefile.ocaml

CC = gcc

CAML_LD_LIBRARY_PATH := /usr/lib/$(shell uname -m)-linux-gnu/:$(CAML_LD_LIBRARY_PATH)
export CAML_LD_LIBRARY_PATH


PKG = -package pprz,glibivy
LINKPKG = $(PKG) -linkpkg -dllpath-pkg pprz,pprzlink

GLIB_CFLAGS  = -Wall -fPIC $(shell pkg-config glib-2.0 --cflags)
GLIB_LDFLAGS = $(shell pkg-config glib-2.0 --libs)

#GLIBIVY_CFLAGS  = -Wall -fPIC $(shell pkg-config --cflags ivy-glib)
#GLIBIVY_LDFLAGS = $(shell pkg-config ivy-glib --libs)

# apparently on OSX `sdl-config --libs` also has -lSDLmain which we don't want
SDL_LDIRS = $(shell pkg-config sdl --libs-only-L)
SDL_LIBS = -lSDL
# we do however need -lSDLmain for test_stick, which is just c
SDL_LDFLAGS = $(shell sdl-config --libs)

# apparently on OSX `sdl-config --libs` also has -lSDLmain which we don't want
ML_SDL_LFLAGS = $(foreach u,$(SDL_LDIRS),-ccopt $(u)) $(foreach u,$(SDL_LIBS),-cclib $(u)) libSDL.so
ML_SDL_LFLAGS += -dllpath ${PAPARAZZI_SRC}/sw/ground_segment/joystick
INCLUDES += -I $(shell ocamlc -where)

SDL_STICK_DEPS = sdl_stick.o ml_sdl_stick.o
# apparently on OSX you cannot dynamically link with SDL, see ocamlsdl readme
UNAME = $(shell uname -s)
MNAME = $(shell uname -m)

ifeq ($(UNAME),Darwin)
	ML_SDL_OCAMLFLAGS = -custom
	INPUT2IVY_DEPS = $(SDL_STICK_DEPS) input2ivy.cmo
else
# on ARM also link in custom mode for now...
	ifneq (,$(findstring arm,$(MNAME)))
		ML_SDL_OCAMLFLAGS = -custom
		INPUT2IVY_DEPS = $(SDL_STICK_DEPS) input2ivy.cmo
	else
		ML_SDL_OCAMLFLAGS =
		INPUT2IVY_DEPS = sdl_stick.so input2ivy.cmo
	endif
endif

all: test_stick input2ivy

test_stick: test_sdl_stick.o
	@echo BUILD $@
	$(Q)$(CC) -g -O2 -DSTICK_DBG $(GLIB_CFLAGS) -o $@ $^ sdl_stick.c $(GLIB_LDFLAGS) $(SDL_LDFLAGS)

input2ivy: $(LIBPPRZCMA) $(LIBPPRZLINKCMA) $(INPUT2IVY_DEPS)
	@echo OL $@
	$(Q)$(OCAMLC) -o $@ $(LINKPKG) $^ $(ML_SDL_OCAMLFLAGS) $(ML_SDL_LFLAGS)


sdl_stick.so : $(SDL_STICK_DEPS)
	@echo BUILD $@
	$(Q)$(CC) -shared -o $@ $^

%.o : %.c
	@echo CC $@
	$(Q)$(CC) -c -O2 $(GLIB_CFLAGS) $(INCLUDES) $<

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) -c $(PKG) $<

clean:
	$(Q)rm -f *~ core *.o *.bak .depend test*stick *.cmo *.cmi input2ivy

.PHONY: all clean

#
# Dependencies
#

.depend: Makefile
	@echo DEPEND $@
	$(Q)$(OCAMLDEP) -I $(LIBPPRZDIR) *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
