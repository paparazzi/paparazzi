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

#
# NOTE: due to a bug in older ocamlbuild versions the build-dir can NOT be absolute
# see http://caml.inria.fr/mantis/view.php?id=5503
#
# This means that for OCaml versions < 4, the PAPARAZZI_SRC must equal PAPARAZZI_HOME
#
OCAML_VERSION := $(shell ocamlc -version)
OCAML_MAJOR := $(shell echo $(OCAML_VERSION) | cut -f1 -d.)
ifeq ($(shell test $(OCAML_MAJOR) -lt 4; echo $$?),0)
# need to use an old buggy ocaml(build) version: build-dir MUST be relative
BUILDDIR ?= ../../../var/build/ocaml/pprz
else
# using a recent ocaml(build) version without THAT particular bug,
# but build-dir MUST NOT contain ..
# so use some magic here to get a normalized absolute path if PAPARAZZI_HOME is not set
PAPARAZZI_HOME ?= $(realpath $(dir $(lastword $(MAKEFILE_LIST)))/../../..)
BUILDDIR ?= $(PAPARAZZI_HOME)/var/build/ocaml/pprz
endif

INSTALL_FLAGS ?=
DESTDIR ?=
ifneq ($(DESTDIR),)
INSTALL_FLAGS += -destdir $(DESTDIR)
endif

INSTALL_FILES = $(shell ls $(BUILDDIR)/*.so $(BUILDDIR)/*.a $(BUILDDIR)/*.mli $(BUILDDIR)/*.cm*)


all: byte native

byte: gen_papget_editors _tags META
	@echo Build bytecode lib
	$(Q)test -d $(BUILDDIR) || mkdir -p $(BUILDDIR)
	$(Q)$(OCAMLBUILD) -build-dir $(BUILDDIR) lib-pprz.cma xlib-pprz.cma
	$(Q)cp META $(BUILDDIR)

# byte with statically linked libs
static: gen_papget_editors _tags META
	@echo Build bytecode lib with static linking
	$(Q)test -d $(BUILDDIR) || mkdir -p $(BUILDDIR)
	$(Q)$(OCAMLBUILD) -build-dir $(BUILDDIR) -tag static lib-pprz.cma xlib-pprz.cma
	$(Q)cp META $(BUILDDIR)

native: gen_papget_editors _tags META
	@echo Build native lib
	$(Q)test -d $(BUILDDIR) || mkdir -p $(BUILDDIR)
	$(Q)$(OCAMLBUILD) -build-dir $(BUILDDIR) lib-pprz.cmxa xlib-pprz.cmxa
	$(Q)cp META $(BUILDDIR)

install: byte native META
	@echo INSTALL
	$(Q)ocamlfind install $(INSTALL_FLAGS) pprz META $(INSTALL_FILES)

tests:
	$(OCAMLBUILD) test/test_latlong.byte
	$(OCAMLBUILD) test/test_logpprz.byte

#
# INTERNAL target from here on...
#
# replacing LABLGTK2GNOMECANVAS in templates with lablgtk2-gnome.gnomecanvas or lablgtk2.gnomecanvas
#
LABLGTK2GNOMECANVAS := $(shell ocamlfind query -p-format lablgtk2-gnome.gnomecanvas 2>/dev/null)
ifeq ($(LABLGTK2GNOMECANVAS),)
LABLGTK2GNOMECANVAS := $(shell ocamlfind query -p-format lablgtk2.gnomecanvas 2>/dev/null)
endif
ifeq ($(LABLGTK2GNOMECANVAS),)
$(error Found neither lablgtk2-gnome.gnomecanvas nor lablgtk2.gnomecanvas package)
endif

NETCLIENT_VER := $(shell ocamlfind query -format '%v' netclient)
NETCLIENT_MAJOR := $(shell echo $(NETCLIENT_VER) | cut -f1 -d.)
ifeq ($(shell test $(NETCLIENT_MAJOR) -ge 4; echo $$?),0)
NETCLIENT_DEF = NETCLIENT_V_4
else
NETCLIENT_DEF = NETCLIENT_V_3
endif

_tags: _tags.template
	@echo CREATE $@
	$(shell sed -e 's/LABLGTK2GNOMECANVAS/$(LABLGTK2GNOMECANVAS)/g' -e 's/NETCLIENT_DEF/$(NETCLIENT_DEF)/g' $< > $@)

META: META.template
	@echo CREATE $@
	$(shell sed -e 's/LABLGTK2GNOMECANVAS/$(LABLGTK2GNOMECANVAS)/g' $< > $@)

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


clean :
	$(Q)rm -f *~ gtk_papget_*.ml META _tags
	$(Q)rm -f *.cm* *.out *.opt .depend *.a *.o *.so expr_parser.ml expr_parser.mli expr_lexer.ml expr_lexer.mli
	$(Q)rm -rf $(BUILDDIR)
	$(Q)ocamlbuild -clean -classic-display -build-dir $(BUILDDIR)

.PHONY: all byte static native tests install install_local gen_papget_editors clean
