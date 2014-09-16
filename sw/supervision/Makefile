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

include ../Makefile.ocaml

UNAME = $(shell uname -s)
ifeq ("$(UNAME)","Darwin")
	MKTEMP = gmktemp
else
	MKTEMP = mktemp
endif

LABLGTK2GNOMEUI = $(shell ocamlfind query -p-format lablgtk2-gnome.gnomeui 2>/dev/null)
ifeq ($(LABLGTK2GNOMEUI),)
LABLGTK2GNOMEUI = $(shell ocamlfind query -p-format lablgtk2.gnomeui 2>/dev/null)
endif

INCLUDES =
XPKG = -package pprz.xlib,$(LABLGTK2GNOMEUI)
XLINKPKG = $(XPKG) -linkpkg -dllpath-pkg pprz.xlib

PAPARAZZICENTERCMO = gtk_pc.cmo gtk_process.cmo pc_common.cmo pc_control_panel.cmo pc_aircraft.cmo paparazzicenter.cmo

all: paparazzicenter

paparazzicenter : $(PAPARAZZICENTERCMO) $(LIBPPRZCMA) $(XLIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(XLINKPKG) gtkInit.cmo $^

gtk_pc.ml : paparazzicenter.glade
	@echo GLADE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)grep -v invisible_char $< > $($@_TMP)
	$(Q)lablgladecc2 -hide-default -root window $($@_TMP) > $@
	$(Q)rm -f $($@_TMP)

gtk_process.ml : paparazzicenter.glade
	@echo GLADE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)grep -v invisible_char $< > $($@_TMP)
	$(Q)lablgladecc2 -hide-default -root hbox_program $($@_TMP) | grep -B 1000000 "  end" > $@
	$(Q)rm -f $($@_TMP)

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $(XPKG) $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $(XPKG) $<

pc_common.cmo: gtk_process.cmo

paparazzicenter.cmo : gtk_pc.cmo

clean:
	$(Q)rm -f *.cm* gtk_pc.ml gtk_process.ml .depend paparazzicenter

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
