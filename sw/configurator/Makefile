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

OCAMLC=ocamlc -g -I +labltk -I +lablgtk2 -I ../lib/ocaml


SRC = env.ml tty.ml varXml.ml console.ml tkXml.ml flasher.ml notebook.ml welcome.ml hardware.ml radio.ml servos.ml adc.ml infrared.ml attitude.ml autopilot.ml airframe.ml flightplan.ml upload.ml simulator.ml monitor.ml logalizer.ml main.ml
CMO = $(SRC:.ml=.cmo)

all : configurator medit.out

configurator : $(CMO)
	$(OCAMLC) -custom -o $@ unix.cma str.cma xml-light.cma labltk.cma jpflib.cma lib.cma $^

medit.out : medit.cmo
	$(OCAMLC) -o $@ str.cma unix.cma xml-light.cma glibivy-ocaml.cma -I +lablgtk2 lablgtk.cma lablgnomecanvas.cma gtkInit.cmo lib.cma $^

%.cmo : %.ml
	$(OCAMLC) -c $<

%.cmi : %.mli
	$(OCAMLC) $<

%.cmi : %.ml
	$(OCAMLC) $<


%.i : %.ml
	$(OCAMLC) -c -i $<

clean :
	rm -f *~ *.cm* *.out configurator medit .depend


#
# Dependencies
#

.depend:
	ocamldep *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif

include ../../conf/Makefile.local
