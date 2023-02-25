# Hey Emacs, this is a -*- makefile -*-
#
#   Copyright (C) 2003 Pascal Brisset, Antoine Drouin
#   Copyright (C) 2022 Gautier Hattenberger
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


Q=@

include ../../../Makefile.ocaml

# verbose ocamlmklib: Print commands before executing them
#VERBOSITY = -verbose
VERBOSITY =

UNAME = $(shell uname -s)
ifeq ("$(UNAME)","Darwin")
	MKTEMP = gmktemp
else
	MKTEMP = mktemp
endif

LABLGTK2GNOMECANVAS = $(shell ocamlfind query -p-format lablgtk2-gnome.gnomecanvas 2>/dev/null)
ifeq ($(LABLGTK2GNOMECANVAS),)
LABLGTK2GNOMECANVAS = $(shell ocamlfind query -p-format lablgtk2.gnomecanvas 2>/dev/null)
endif

INCLUDES=
PKGCOMMON=pprzlink
XINCLUDES=
XPKGCOMMON=pprzlink,xml-light,glibivy,$(LABLGTK2GNOMECANVAS),lablgtk2.glade

PKG = -package pprzlink,pprz.xlib
LINKPKG = $(PKG) -linkpkg -dllpath-pkg pprz.xlib,pprzlink

XSRC = contrastLabel.ml acIcon.ml wind_sock.ml gtk_papget_editor.ml gtk_papget_text_editor.ml gtk_papget_gauge_editor.ml gtk_papget_led_editor.ml papget_common.ml papget_renderer.ml papget.ml mapCanvas.ml mapWaypoints.ml mapTrack.ml mapGoogle.ml mapIGN.ml ml_gtk_drag.o xmlEdit.ml mapFP.ml
XCMO = $(XSRC:.ml=.cmo)
XCMX = $(XSRC:.ml=.cmx)

all : byte native
byte : gcslib-pprz.cma
native : gcslib-pprz.cmxa

gcslib-pprz.cma libgcslib-pprz.a: $(XCMO)
	@echo OL $@
	$(Q)$(OCAMLMKLIB) $(VERBOSITY) $(XINCLUDES) -o gcslib-pprz $^

gcslib-pprz.cmxa dllgcslib-pprz.so: $(XCMX)
	@echo OOL $@
	$(Q)$(OCAMLMKLIB) $(VERBOSITY) $(XINCLUDES) -o gcslib-pprz $^

# trying to set correct dependencies for parallel build
# these are order only depedencies
gcslib-pprz.cma: | libgcslib-pprz.a dllgcslib-pprz.so

gcslib-pprz.cmxa: | libgcslib-pprz.a dllgcslib-pprz.so

%.o : %.c
	@echo OC $<
	$(Q)$(OCAMLC) -ccopt -fPIC $(INCLUDES) -package $(PKGCOMMON) -c $<

$(XCMO) $(XCMX): PKGCOMMON=$(XPKGCOMMON)


GTKCFLAGS := $(shell pkg-config --cflags gtk+-2.0) -DGTK_DISABLE_DEPRECATED
ml_gtk_drag.o : ml_gtk_drag.c
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -package $(PKGCOMMON) -c -ccopt "$(GTKCFLAGS)" $<

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) $(PKG) -c $<

%.cmx : %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(INCLUDES) $(PKG) -c $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $(XINCLUDES) $(INCLUDES) $(PKG) $<

gtk_papget_editor.ml : widgets.glade
	@echo GLADE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)grep -v invisible_char $< > $($@_TMP)
	$(Q)lablgladecc2 -root papget_editor -hide-default $($@_TMP) | grep -B 1000000 "  end" > $@
	$(Q)rm -f $($@_TMP)

gtk_papget_text_editor.ml : widgets.glade
	@echo GLADE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)grep -v invisible_char $< > $($@_TMP)
	$(Q)lablgladecc2 -root table_text_editor -hide-default $($@_TMP) | grep -B 1000000 "  end" > $@
	$(Q)rm -f $($@_TMP)

gtk_papget_gauge_editor.ml : widgets.glade
	@echo GLADE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)grep -v invisible_char $< > $($@_TMP)
	$(Q)lablgladecc2 -root table_gauge_editor -hide-default $($@_TMP) | grep -B 1000000 "  end" > $@
	$(Q)rm -f $($@_TMP)

gtk_papget_led_editor.ml : widgets.glade
	@echo GLADE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)grep -v invisible_char $< > $($@_TMP)
	$(Q)$(Q)lablgladecc2 -root table_led_editor -hide-default $($@_TMP) | grep -B 1000000 "  end" > $@
	$(Q)rm -f $($@_TMP)

clean :
	$(Q)rm -f *~ *.cm* *.out *.opt .depend *.a *.o *.so gtk_papget_*.ml

.PHONY: all byte native clean

#
# Dependencies
#

.depend: Makefile
	@echo DEPEND $@
	$(Q)$(OCAMLDEP) $(XSRC) *.mli >> .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
