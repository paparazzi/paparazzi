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


CMO = env_conf.cmo notebook.cmo tty.cmo varXml.cmo console.cmo tkXml.cmo flasher.cmo welcome.cmo hardware.cmo radio.cmo servos.cmo adc.cmo infrared.cmo attitude.cmo autopilot.cmo airframe.cmo flightplan.cmo upload.cmo simulator.cmo monitor.cmo logalizer.cmo main.ml

ABS_CMO = $(CMO:%=$$PAPARAZZI_SRC/sw/configurator/%)

all : configurator medit

configurator : $(CMO)
	$(OCAMLC) -custom -o /dev/null unix.cma str.cma xml-light.cma labltk.cma jpflib.cma lib-pprz.cma $^ # To check
	cat ../../pprz_src_test.sh > $@
	echo 'ocaml -I +labltk -I $$PAPARAZZI_SRC/sw/lib/ocaml -I $$PAPARAZZI_SRC/sw/configurator unix.cma str.cma xml-light.cma labltk.cma jpflib.cma lib-pprz.cma $(ABS_CMO) $$*' >> $@
	chmod a+x $@

medit : medit.ml
	$(OCAMLC) -o /dev/null str.cma unix.cma xml-light.cma glibivy-ocaml.cma -I +lablgtk2 lablgtk.cma lablgnomecanvas.cma gtkInit.cmo lib-pprz.cma $^ # To check
	cat ../../pprz_src_test.sh > $@
	echo 'lablgtk2 -I $$PAPARAZZI_SRC/sw/lib/ocaml str.cma unix.cma xml-light.cma glibivy-ocaml.cma lib-pprz.cma $$PAPARAZZI_SRC/sw/configurator/$< $$*' >> $@
	chmod a+x $@



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
