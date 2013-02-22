# Hey Emacs, this is a -*- makefile -*-
#
#   Copyright (C) 2003-2006 Pascal Brisset, Antoine Drouin
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

# Quiet
Q=@

LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
  FPIC = -fPIC
else
  FPIC =
endif


include ../../../conf/Makefile.local
include ../../Makefile.ocaml
CONF = ../../../conf
VAR = ../../../var

INCLUDES= -I $(LIBPPRZDIR) -I ../multimon
PKGCOMMON=unix,str,lablgtk2,glibivy,xml-light,netstring,netclient

LIBMULTIMONCMA=../multimon/multimon.cma
LIBMULTIMONDLL= multimon.cma -dllpath $(PAPARAZZI_SRC)/sw/ground_segment/multimon

SERVERCMO = server_globals.cmo aircraft.cmo wind.cmo airprox.cmo kml.cmo fw_server.ml rotorcraft_server.ml server.cmo
SERVERCMX = $(SERVERCMO:.cmo=.cmx)


all: link server messages settings dia diadec $(VAR)/boa.conf ivy_tcp_aircraft ivy_tcp_controller broadcaster ivy2udp ivy_serial_bridge

clean:
	$(Q)rm -f link server messages settings dia diadec *.bak *~ core *.o .depend *.opt *.out *.cm* ivy_tcp_aircraft ivy_tcp_controller broadcaster ivy2udp ivy_serial_bridge gpsd2ivy c_ivy_client_example_1 c_ivy_client_example_2 c_ivy_client_example_3


$(VAR)/boa.conf :$(CONF)/boa.conf
	mkdir -p $(VAR)
	sed 's|PAPARAZZI_HOME|$(PAPARAZZI_HOME)|' < $< > $@


messages : messages.cmo $(XLIBPPRZCMA) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLFIND) $(OCAMLC) $(INCLUDES) -o $@ -package $(PKGCOMMON) -linkpkg $(OCAMLDLL) $(LIBPPRZCMA) gtkInit.cmo $<

settings : settings.cmo $(XLIBPPRZCMA) $(LIBPPRZCMA) ../cockpit/page_settings.cmo
	@echo OL $@
	$(Q)$(OCAMLFIND) $(OCAMLC) $(INCLUDES) -o $@ -package $(PKGCOMMON) -linkpkg lablglade.cma lablgnomecanvas.cma gtkInit.cmo $(OCAMLDLL) $(LIBPPRZCMA) $(OCAMLXDLL) $(XLIBPPRZCMA) -I ../cockpit gtk_save_settings.cmo saveSettings.cmo page_settings.cmo $<

server : $(SERVERCMO) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLFIND) $(OCAMLC) $(INCLUDES) -o $@ -package $(PKGCOMMON) -linkpkg $(OCAMLDLL) $(LIBPPRZCMA) $(SERVERCMO)

server.opt :  $(SERVERCMX) $(LIBPPRZCMXA)
	@echo OOL $@
	$(Q)$(OCAMLFIND) $(OCAMLOPT) $(INCLUDES) -o $@ -package $(PKGCOMMON) -linkpkg $(OCAMLDLL) $(LIBPPRZCMXA) $(SERVERCMX)

link : link.cmo $(LIBMULTIMONCMA) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLFIND) $(OCAMLC) $(INCLUDES) -o $@ -package $(PKGCOMMON) -linkpkg $(OCAMLDLL) $(LIBPPRZCMA) $(LIBMULTIMONDLL) $<


ivy_tcp_aircraft : ivy_tcp_aircraft.cmo  $(LIBMULTIMONCMA) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLFIND) $(OCAMLC) $(INCLUDES) -o $@ -package $(PKGCOMMON) -linkpkg $(OCAMLDLL) $(LIBPPRZCMA) $(LIBMULTIMONDLL) $<


ivy_tcp_controller : ivy_tcp_controller.cmo  $(LIBMULTIMONCMA) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLFIND) $(OCAMLC) $(INCLUDES) -o $@ -package $(PKGCOMMON) -linkpkg $(OCAMLDLL) $(LIBPPRZCMA) $(LIBMULTIMONDLL) $<


broadcaster : broadcaster.cmo $(LIBMULTIMONCMA) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLFIND) $(OCAMLC) $(INCLUDES) -o $@ -package $(PKGCOMMON) -linkpkg $(OCAMLDLL) $(LIBPPRZCMA) $(LIBMULTIMONDLL) $<


ivy2udp : ivy2udp.cmo $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLFIND) $(OCAMLC) $(INCLUDES) -o $@ -package $(PKGCOMMON) -linkpkg $(OCAMLDLL) $(LIBPPRZCMA) $<


dia : dia.cmo $(LIBMULTIMONCMA) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLFIND) $(OCAMLC) $(INCLUDES) -o $@ -package $(PKGCOMMON) -linkpkg $(OCAMLDLL) $(LIBPPRZCMA) $(LIBMULTIMONDLL) $<


diadec : diadec.cmo $(LIBMULTIMONCMA) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLFIND) $(OCAMLC) $(INCLUDES) -o $@ -package $(PKGCOMMON) -linkpkg $(OCAMLDLL) $(LIBPPRZCMA) $(LIBMULTIMONDLL) $<


150m : 150m.cmo $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLFIND) $(OCAMLC) $(INCLUDES) -o $@ -package $(PKGCOMMON) -linkpkg $(OCAMLDLL) $(LIBPPRZCMA) gtkInit.cmo $<

settings.cmo : INCLUDES += -I ../cockpit
settings.cmo : ../cockpit/page_settings.cmi

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLFIND) $(OCAMLC) $(INCLUDES) -package $(PKGCOMMON) -c $<

%.cmx : %.ml
	@echo OOC $<
	$(Q)$(OCAMLFIND) $(OCAMLOPT) $(INCLUDES) -package $(PKGCOMMON) -c $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLFIND) $(OCAMLC) $(INCLUDES) -package $(PKGCOMMON) $<


CC = gcc
GLIB_CFLAGS  = -Wall  `pkg-config glib-2.0 --cflags` $(FPIC)
GLIB_LDFLAGS =  `pkg-config glib-2.0 --libs` -lglibivy -lpcre $(FPIC)
GTK_CFLAGS  = -Wall  `pkg-config gtk+-2.0 --cflags` $(FPIC)
GTK_LDFLAGS =  `pkg-config gtk+-2.0 --libs` -lglibivy -lpcre $(FPIC)

UNAME = $(shell uname -s)

ifeq ("$(UNAME)","Darwin")
  C_LIBRARYS = $(shell if test -d /opt/paparazzi/lib; then echo "-L/opt/paparazzi/lib"; elif test -d /opt/local/lib; then echo "-L/opt/local/lib"; fi)
  C_INCLUDES = $(shell if test -d /opt/paparazzi/include; then echo "-I/opt/paparazzi/include"; elif test -d /opt/local/include; then echo "-I/opt/local/include"; fi)
endif

gpsd2ivy: gpsd2ivy.c
	$(CC) $(GLIB_CFLAGS) $(C_LIBRARYS) $(C_INCLUDES) -o $@ $< $(GLIB_LDFLAGS) -lgps

c_ivy_client_example_1: c_ivy_client_example_1.c
	$(CC) $(GLIB_CFLAGS) $(C_LIBRARYS) $(C_INCLUDES) -o $@ $< $(GLIB_LDFLAGS)

c_ivy_client_example_2: c_ivy_client_example_2.c
	$(CC) $(GLIB_CFLAGS) $(C_LIBRARYS) $(C_INCLUDES) -o $@ $< $(GLIB_LDFLAGS)

c_ivy_client_example_3: c_ivy_client_example_3.c
	$(CC) $(GTK_CFLAGS) $(C_LIBRARYS) $(C_INCLUDES) -o $@ $< $(GTK_LDFLAGS)

ivy_serial_bridge: ivy_serial_bridge.c
	$(CC) $(GTK_CFLAGS) $(C_LIBRARYS) $(C_INCLUDES) -o $@ $< $(GTK_LDFLAGS)


.PHONY: all clean

#
# Dependencies
#

.depend: Makefile
	$(OCAMLDEP) -I $(LIBPPRZDIR) -I ../multimon *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
