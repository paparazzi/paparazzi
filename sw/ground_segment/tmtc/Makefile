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

all: receive messages listen.opt

clean:
	rm -f receive messages *.bak *~ core *.o .depend *.opt *.out *.cm*

OCAMLC = ocamlc
OCAMLOPT = ocamlopt
INCLUDES= -I ../../lib/ocaml -I +lablgtk2

messages: messages.ml
	$(OCAMLC) $(INCLUDES) -o /dev/null unix.cma xml-light.cma glibivy-ocaml.cma -I +lablgtk2 lablgtk.cma gtkInit.cmo str.cma lib-pprz.cma $^ # To check
	cat ../../../pprz_src_test.sh > $@
	echo 'lablgtk2 str.cma ivy-ocaml.cma -I $$PAPARAZZI_SRC/sw/lib/ocaml lib-pprz.cma $$PAPARAZZI_SRC/sw/ground_segment/tmtc/$< $$*' >> $@
	chmod a+x $@


receive : modem.cmo receive.ml
	$(OCAMLC) $(INCLUDES) -o $@ str.cma unix.cma xml-light.cma glibivy-ocaml.cma -I +lablgtk2 lablgtk.cma lib-pprz.cma $^ #To check
	cat ../../../pprz_src_test.sh > $@
	echo 'lablgtk2 str.cma glibivy-ocaml.cma -I $$PAPARAZZI_SRC/sw/lib/ocaml lib-pprz.cma -I $$PAPARAZZI_SRC/sw/ground_segment/tmtc $$PAPARAZZI_SRC/sw/ground_segment/tmtc/modem.cmo $$PAPARAZZI_SRC/sw/ground_segment/tmtc/receive.ml $$*' >> $@
	chmod a+x $@

listen.opt : listen.ml
	$(OCAMLOPT) $(INCLUDES) -o $@ str.cmxa unix.cmxa xml-light.cmxa glibivy-ocaml.cmxa -I +lablgtk2 lablgtk.cmxa lib-pprz.cmxa $^
	strip $@


%.cmo : %.ml
	$(OCAMLC) $(INCLUDES) -c $<
