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

OCAMLC=ocamlc -I +labltk -I ../lib/ocaml


CMO = env_conf.cmo notebook.cmo tty.cmo varXml.cmo console.cmo tkXml.cmo flasher.cmo welcome.cmo hardware.cmo radio.cmo servos.cmo adc.cmo infrared.cmo attitude.cmo autopilot.cmo airframe.cmo flightplan.cmo upload.cmo simulator.cmo monitor.cmo logalizer.cmo main.ml

ABS_CMO = $(CMO:%=$$PAPARAZZI_SRC/sw/configurator/%)

all : configurator

configurator : $(CMO)
	$(OCAMLC) -custom -o $@ labltk.cma lib-pprz.cma jpflib.cma $^


%.cmo : %.ml ../lib/ocaml/lib-pprz.cma
	$(OCAMLC) -c $<

%.cmi : %.mli ../lib/ocaml/lib-pprz.cma
	$(OCAMLC) $<

%.cmi : %.ml
	$(OCAMLC) $<


%.i : %.ml
	$(OCAMLC) -c -i $<

clean :
	rm -f *~ *.cm* *.out configurator .depend


#
# Dependencies
#

.depend:
	ocamldep *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif

include ../../conf/Makefile.local
