# 
#   $Id$
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

include ../../../conf/Makefile.local

all: messages.cmo modem.cmo receive.cmo receive.opt messages.opt

clean:
	rm -f receive *.bak *~ core *.o .depend *.opt *.out *.cm*

OCAMLC = ocamlc
OCAMLOPT = ocamlopt
INCLUDES= -I ../../lib/ocaml -I +lablgtk2

messages.opt : messages.ml
	$(OCAMLOPT) $(INCLUDES) -o $@ unix.cmxa xml-light.cmxa glibivy-ocaml.cmxa -I +lablgtk2 lablgtk.cmxa gtkInit.cmx str.cmxa lib.cmxa $^
	strip $@

messages.run:
	lablgtk2 str.cma -I ../../lib/ocaml ivy-ocaml.cma lib.cma messages.cmo

receive.opt : modem.ml receive.ml
	$(OCAMLOPT) $(INCLUDES) -o $@ str.cmxa unix.cmxa xml-light.cmxa glibivy-ocaml.cmxa -I +lablgtk2 lablgtk.cmxa lib.cmxa $^
	strip $@

listen.opt : listen.ml
	$(OCAMLOPT) $(INCLUDES) -o $@ str.cmxa unix.cmxa xml-light.cmxa glibivy-ocaml.cmxa -I +lablgtk2 lablgtk.cmxa lib.cmxa $^
	strip $@

receive.out : modem.ml receive.ml
	$(OCAMLC) -g $(INCLUDES) -o $@ str.cma unix.cma xml-light.cma glibivy-ocaml.cma -I +lablgtk2 lablgtk.cma lib.cma $^


receive.run:
	lablgtk2 str.cma -I ../../lib/ocaml ivy-ocaml.cma lib.cma modem.cmo receive.ml


%.cmo : %.ml
	$(OCAMLC) $(INCLUDES) -c $<
