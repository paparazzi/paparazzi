# Hey Emacs, this is a -*- makefile -*-
#
#   Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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

OCAMLBUILD=ocamlbuild -use-ocamlfind

LABLGTK2GNOMECANVAS = $(shell ocamlfind query -p-format lablgtk2-gnome.gnomecanvas 2>/dev/null)
ifeq ($(LABLGTK2GNOMECANVAS),)
LABLGTK2GNOMECANVAS = $(shell ocamlfind query -p-format lablgtk2.gnomecanvas 2>/dev/null)
endif

#
# FIXME, do this check in myocamlbuild or try to pass as option
#
NETCLIENT_VER := $(shell ocamlfind query -format '%v' netclient)
NETCLIENT_MAJOR := $(shell echo $(NETCLIENT_VER) | cut -f1 -d.)
ifeq ($(shell test $(NETCLIENT_MAJOR) -ge 4; echo $$?),0)
CAMLP4_DEFS = -DNETCLIENT_V_4
endif



all: byte native

byte: gen_papget_editors META.pprz
	@echo OB byte
	$(Q)$(OCAMLBUILD) lib-pprz.cma xlib-pprz.cma

native: gen_papget_editors META.pprz
	@echo OB native
	$(Q)$(OCAMLBUILD) lib-pprz.cmxa xlib-pprz.cmxa

tests:
	$(OCAMLBUILD) test/test_latlong.byte
	$(OCAMLBUILD) test/test_logpprz.byte


gen_papget_editors: gtk_papget_editor.ml gtk_papget_text_editor.ml gtk_papget_gauge_editor.ml gtk_papget_led_editor.ml

UNAME = $(shell uname -s)
ifeq ("$(UNAME)","Darwin")
	MKTEMP = gmktemp
else
	MKTEMP = mktemp
endif

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

META.pprz: META.pprz.template
	@echo COPY $<
	$(shell cp $< META.pprz && sed -i 's/LABLGTK2GNOMECANVAS/$(LABLGTK2GNOMECANVAS)/g' META.pprz)

clean :
	$(Q)rm -f *~ *.cm* *.out *.opt .depend *.a *.o *.so gtk_papget_*.ml expr_parser.ml expr_parser.mli expr_lexer.ml expr_lexer.mli META.pprz
	$(Q)ocamlbuild -clean -classic-display

.PHONY: all byte native tests gen_papget_editors clean
