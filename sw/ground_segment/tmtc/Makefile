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

include ../../../conf/Makefile.local
CONF = ../../../conf
VAR = ../../../var

all: link server messages settings dia diadec $(VAR)/boa.conf

clean:
	rm -f link server messages settings dia diadec *.bak *~ core *.o .depend *.opt *.out *.cm*

OCAMLC = ocamlc
OCAMLOPT = ocamlopt
OCAMLLIB = ../../lib/ocaml
INCLUDES= -I $(OCAMLLIB) -I ../multimon -I +lablgtk2 -I +xml-light
LIBPPRZCMA=$(OCAMLLIB)/lib-pprz.cma

SERVERCMO = aircraft.cmo wind.cmo airprox.cmo kml.cmo server.cmo
SERVERCMX = $(SERVERCMO:.cmo=.cmx)

$(VAR)/boa.conf :$(CONF)/boa.conf
	mkdir -p $(VAR)
	sed 's|PAPARAZZI_HOME|$(PAPARAZZI_HOME)|' < $< > $@
	chmod a+x boa


messages : messages.cmo ../../lib/ocaml/xlib-pprz.cma ../../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma gtkInit.cmo $<


settings : settings.cmo ../../lib/ocaml/xlib-pprz.cma ../../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -I ../cockpit -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma gtkInit.cmo pages.cmo $<


server : $(SERVERCMO) ../../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma $(SERVERCMO)

server.opt :  $(SERVERCMX) ../../lib/ocaml/lib-pprz.cmxa
	@echo OOL $@
	$(Q)$(OCAMLOPT) $(INCLUDES) -o $@ str.cmxa unix.cmxa xml-light.cmxa lablgtk.cmxa glibivy-ocaml.cmxa lib-pprz.cmxa $(SERVERCMX)

LINKCMO=modem.cmo link.cmo

link : $(LINKCMO)  ../../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma multimon.cma $(LINKCMO)


dia : dia.cmo ../multimon/multimon.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma multimon.cma $^


diadec : diadec.cmo ../multimon/multimon.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma multimon.cma $^


ihm :  ihm.cmo
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma lablgtk.cma glibivy-ocaml.cma lib-pprz.cma $^


settings.cmo : INCLUDES += -I ../cockpit

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

%.cmx : %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(INCLUDES) -c $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) $<


#
# Dependencies
#

.depend: Makefile
	ocamldep -I ../../lib/ocaml *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif

