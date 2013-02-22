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
TOOLSDIR = ../../tools

CC = gcc

LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
  FPIC=-fpic
else
  FPIC=
endif

OCAMLINCLUDES= -I $(LIBPPRZDIR) -I $(TOOLSDIR)
PKGCOMMON=unix,str,lablgtk2,glibivy,xml-light
LIBPPRZCMA=$(LIBPPRZDIR)/lib-pprz.cma

GLIB_CFLAGS  = -Wall $(shell pkg-config glib-2.0 --cflags) $(FPIC)
GLIB_LDFLAGS = $(shell pkg-config glib-2.0 --libs) -lglibivy

# apparently on OSX `sdl-config --libs` also has -lSDLmain which we don't want
SDL_LDIRS = $(shell pkg-config sdl --libs-only-L)
SDL_LIBS = -lSDL
SDL_LDFLAGS = $(SDL_LDIRS) $(SDL_LIBS)

# apparently on OSX `sdl-config --libs` also has -lSDLmain which we don't want
ML_SDL_LFLAGS = $(foreach u,$(SDL_LDIRS),-ccopt $(u)) $(foreach u,$(SDL_LIBS),-cclib $(u)) libSDL.so
INCLUDES += -I `ocamlc -where`


all: test_stick input2ivy

test_stick: test_sdl_stick.o
	@echo BUILD $@
	$(Q)$(CC) -g -O2 -DSTICK_DBG $(GLIB_CFLAGS) -o $@ $^ sdl_stick.c $(GLIB_LDFLAGS) $(SDL_LDFLAGS)

input2ivy: sdl_stick.so input2ivy.cmo
	@echo OL $@
	$(Q)$(OCAMLFIND) $(OCAMLC) $(OCAMLINCLUDES) -o $@ -package $(PKGCOMMON) -linkpkg $(LIBPPRZCMA) $(TOOLSDIR)/fp_proc.cmo $^ $(ML_SDL_LFLAGS)

# dependency of input2ivy
input2ivy: $(LIBPPRZCMA) $(TOOLSDIR)/fp_proc.cmo

sdl_stick.so : sdl_stick.o ml_sdl_stick.o
	@echo BUILD $@
	$(Q)$(CC) -shared -o $@ $^

%.o : %.c
	@echo CC $@
	$(Q)$(CC) -c -O2 $(GLIB_CFLAGS) $(INCLUDES) $<

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLFIND) $(OCAMLC) $(OCAMLINCLUDES) -c -package $(PKGCOMMON) $<

clean:
	$(Q)rm -f *~ core *.o *.bak .depend test*stick *.cmo *.cmi input2ivy

.PHONY: all clean

#
# Dependencies
#

.depend: Makefile
	$(OCAMLDEP) -I $(LIBPPRZDIR) *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
