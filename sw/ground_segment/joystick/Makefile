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

OCAMLINCLUDES= -I $(LIBPPRZDIR) $(shell ocamlfind query -r -i-format lablgtk2) $(shell ocamlfind query -r -i-format xml-light) -I $(TOOLSDIR)
PKGCOMMON=unix,str,lablgtk2,ivy,xml-light
LIBPPRZCMA=$(LIBPPRZDIR)/lib-pprz.cma

# apparently on OSX `sdl-config --libs` also has -lSDLmain which we don't want
#ML_SDL_LFLAGS = $(foreach u,$(shell sdl-config --libs),-cclib $(u))
ML_SDL_LFLAGS = $(foreach u,$(shell pkg-config sdl --libs-only-L) -lSDL,-cclib $(u))
INCLUDES += -I `ocamlc -where`

all: test_stick input2ivy

test_stick: test_sdl_stick.o
	gcc -g -O2 -Wall -DSTICK_DBG `pkg-config glib-2.0 --cflags` -o $@ $^ sdl_stick.c `pkg-config glib-2.0 --libs` -lglibivy `sdl-config --libs`

input2ivy: sdl_stick.o ml_sdl_stick.o input2ivy.cmo
	@echo OL $@
	$(Q)$(OCAMLC) -make-runtime -o i2i_run sdl_stick.o ml_sdl_stick.o $(ML_SDL_LFLAGS)
	$(Q)$(OCAMLFIND) $(OCAMLC) $(OCAMLINCLUDES) -use-runtime i2i_run -o $@ -package $(PKGCOMMON) -linkpkg $(OCAMLDLL) $(LIBPPRZCMA) $(TOOLSDIR)/fp_proc.cmo

# dependency of input2ivy
input2ivy: $(LIBPPRZCMA) $(TOOLSDIR)/fp_proc.cmo

%.o : %.c
	gcc -c -O2 -Wall `pkg-config glib-2.0 --cflags` $(INCLUDES) $<

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(OCAMLINCLUDES) -c $<

clean:
	rm -f *~ core *.o *.bak .depend test*stick *.cmo *.cmi input2ivy

.PHONY: all clean

#
# Dependencies
#

.depend: Makefile
	$(OCAMLDEP) -I $(LIBPPRZDIR) *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
