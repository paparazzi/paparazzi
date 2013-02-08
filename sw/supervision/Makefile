# Hey Emacs, this is a -*- makefile -*-
#
#   Copyright (C) 2004-2012 Pascal Brisset, Antoine Drouin
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

OCAMLC = ocamlc
OCAMLOPT = ocamlopt
OCAMLDEP = ocamldep
LIBPPRZDIR = ../lib/ocaml
INCLUDES= -I $(LIBPPRZDIR) $(shell ocamlfind query -r -i-format lablgtk2) $(shell ocamlfind query -r -i-format xml-light)
LIBPPRZCMA=$(LIBPPRZDIR)/lib-pprz.cma
PAPARAZZICENTERCMO = gtk_pc.cmo gtk_process.cmo pc_common.cmo pc_aircraft.cmo pc_control_panel.cmo paparazzicenter.cmo

# default directory for temporary files
TMPDIR ?= /tmp

all: paparazzicenter

paparazzicenter : $(PAPARAZZICENTERCMO) $(LIBPPRZDIR)/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma lablglade.cma gtkInit.cmo lib-pprz.cma xlib-pprz.cma lablgnomeui.cma $^

gtk_pc.ml : paparazzicenter.glade
	grep -v invisible_char $< > $(TMPDIR)/$<
	lablgladecc2 -hide-default -root window $(TMPDIR)/$< > $@

gtk_process.ml : paparazzicenter.glade
	grep -v invisible_char $< > $(TMPDIR)/$<
	lablgladecc2 -hide-default -root hbox_program $(TMPDIR)/$< | grep -B 1000000 "  end" > $@

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) $<

pc_common.cmo: gtk_process.cmo

paparazzicenter.cmo : gtk_pc.cmo

clean:
	\rm -f *.cm* gtk_pc.ml gtk_process.ml .depend paparazzicenter

.PHONY: all clean

#
# Dependencies
#

.depend: Makefile
	$(OCAMLDEP) -I $(LIBPPRZDIR) *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
