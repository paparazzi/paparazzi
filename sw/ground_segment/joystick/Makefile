#
#   $Id$
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
#

# Quiet compilation
Q=@

OCAMLC = ocamlc
OCAMLLIB = ../../lib/ocaml
TOOLSDIR = ../../tools
OCAMLINCLUDES= -I $(OCAMLLIB) $(shell ocamlfind query -r -i-format lablgtk2) $(shell ocamlfind query -r -i-format xml-light) -I $(TOOLSDIR)
LIBPPRZCMA=$(OCAMLLIB)/lib-pprz.cma

all: test_stick input2ivy

test_stick: test_stick.o
	gcc -g -O2 -Wall -DSTICK_DBG `pkg-config glib-2.0 --cflags` -o $@ $^ usb_stick.c `pkg-config glib-2.0 --libs` -lglibivy

input2ivy: usb_stick.o ml_usb_stick.o input2ivy.cmo
	$(OCAMLC) $(OCAMLINCLUDES) -custom -o $@ unix.cma str.cma glibivy-ocaml.cma xml-light.cma lib-pprz.cma lablgtk.cma $(TOOLSDIR)/fp_proc.cmo $^


%.o : %.c
	gcc -c -O2 -Wall `pkg-config glib-2.0 --cflags` $<

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(OCAMLINCLUDES) -c $<

clean:
	rm -f *~ core *.o *.bak .depend test_stick *.cmo *.cmi input2ivy
