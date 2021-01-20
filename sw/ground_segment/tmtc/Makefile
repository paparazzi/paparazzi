#
#   $Id$
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
CONF = ../../../conf
VAR = ../../../var

all: link server messages settings dia diadec $(VAR)/boa.conf ivy_tcp_aircraft ivy_tcp_controller broadcaster ivy2udp ivy_serial_bridge gpsd2ivy

clean:
	rm -f link server messages settings dia diadec *.bak *~ core *.o .depend *.opt *.out *.cm* ivy_tcp_aircraft ivy_tcp_controller broadcaster ivy2udp ivy_serial_bridge gpsd2ivy

OCAMLC = ocamlc
OCAMLOPT = ocamlopt
OCAMLLIB = ../../lib/ocaml
OCAMLNETINCLUDES=$(shell ocamlfind query -r -i-format netstring) $(shell ocamlfind query -r -i-format netclient)
OCAMLNETCMA=$(shell ocamlfind query -r -a-format -predicates byte netstring)
INCLUDES= -I $(OCAMLLIB) -I ../multimon $(shell ocamlfind query -r -i-format lablgtk2) $(shell ocamlfind query -r -i-format xml-light) $(OCAMLNETINCLUDES)
LIBPPRZCMA=$(OCAMLLIB)/lib-pprz.cma

SERVERCMO = server_globals.cmo aircraft.cmo wind.cmo airprox.cmo kml.cmo fw_server.ml rotorcraft_server.ml server.cmo
SERVERCMX = $(SERVERCMO:.cmo=.cmx)

$(VAR)/boa.conf :$(CONF)/boa.conf
	mkdir -p $(VAR)
	sed 's|PAPARAZZI_HOME|$(PAPARAZZI_HOME)|' < $< > $@
	chmod a+x boa


messages : messages.cmo ../../lib/ocaml/xlib-pprz.cma ../../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma gtkInit.cmo $<


settings : settings.cmo ../../lib/ocaml/xlib-pprz.cma ../../lib/ocaml/lib-pprz.cma ../cockpit/page_settings.cmo
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -I ../cockpit -o $@ unix.cma str.cma xml-light.cma lablgtk.cma lablglade.cma $(OCAMLNETCMA) glibivy-ocaml.cma lib-pprz.cma lablgnomecanvas.cma xlib-pprz.cma gtkInit.cmo gtk_save_settings.cmo saveSettings.cmo page_settings.cmo $<


server : $(SERVERCMO) ../../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma $(SERVERCMO)

server.opt :  $(SERVERCMX) ../../lib/ocaml/lib-pprz.cmxa
	@echo OOL $@
	$(Q)$(OCAMLOPT) $(INCLUDES) -o $@ str.cmxa unix.cmxa xml-light.cmxa lablgtk.cmxa glibivy-ocaml.cmxa lib-pprz.cmxa $(SERVERCMX)

LINKCMO=link.cmo

link : $(LINKCMO)  ../../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma multimon.cma $(LINKCMO)


ivy_tcp_aircraft : ivy_tcp_aircraft.cmo  ../../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma multimon.cma ivy_tcp_aircraft.cmo


ivy_tcp_controller : ivy_tcp_controller.cmo  ../../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma multimon.cma ivy_tcp_controller.cmo


broadcaster : broadcaster.cmo  ../../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma multimon.cma broadcaster.cmo


ivy2udp : ivy2udp.cmo  ../../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma $<


dia : dia.cmo ../multimon/multimon.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma multimon.cma $^


diadec : diadec.cmo ../multimon/multimon.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma multimon.cma $^


150m : 150m.cmo
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma gtkInit.cmo $^


settings.cmo : INCLUDES += -I ../cockpit
settings.cmo : ../cockpit/page_settings.cmi

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

%.cmx : %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(INCLUDES) -c $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) $<


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


#
# Dependencies
#

.depend: Makefile
	ocamldep -I ../../lib/ocaml *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
