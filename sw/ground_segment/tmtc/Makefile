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
CONF = ../../../conf
VAR = ../../../var

all: hw_modem_listen receive messages $(VAR)/boa.conf
# stereo_demod

clean:
	rm -f hw_modem_listen wavecard_connect receive messages *.bak *~ core *.o .depend *.opt *.out *.cm*

OCAMLC = ocamlc
OCAMLOPT = ocamlopt
INCLUDES= -I ../../lib/ocaml -I +lablgtk2

$(VAR)/boa.conf :$(CONF)/boa.conf
	mkdir -p $(VAR)
	sed 's|PAPARAZZI_HOME|$(PAPARAZZI_HOME)|' < $< > $@
	chmod a+x boa



messages.opt : messages.ml
	$(OCAMLOPT) $(INCLUDES) -o $@ str.cmxa unix.cmxa xml-light.cmxa glibivy-ocaml.cmxa -I +lablgtk2 lablgtk.cmxa gtkInit.cmx lib-pprz.cmxa $^ 
	strip $@

messages : messages.ml
	$(OCAMLC) $(INCLUDES) -o $@ lablgtk.cma glibivy-ocaml.cma lib-pprz.cma $^ #To check
	@cat ../../../pprz_src_test.sh > $@
	@echo 'exec lablgtk2 glibivy-ocaml.cma -I $$PAPARAZZI_SRC/sw/lib/ocaml glibivy-ocaml.cma lib-pprz.cma $$PAPARAZZI_SRC/sw/ground_segment/tmtc/messages.ml $$*' >> $@
	chmod a+x $@


hw_modem_listen : modem.cmo hw_modem_listen.ml ../../lib/ocaml/lib-pprz.cma
	$(OCAMLC) $(INCLUDES) -o $@ lablgtk.cma glibivy-ocaml.cma lib-pprz.cma $^
	@cat ../../../pprz_src_test.sh > $@
	@echo 'exec lablgtk2 -I $$PAPARAZZI_SRC/sw/lib/ocaml glibivy-ocaml.cma lib-pprz.cma -I $$PAPARAZZI_SRC/sw/ground_segment/tmtc $$PAPARAZZI_SRC/sw/ground_segment/tmtc/modem.cmo $$PAPARAZZI_SRC/sw/ground_segment/tmtc/hw_modem_listen.ml $$*' >> $@
	@chmod a+x $@

stereo_demod : ../multimon/multimon.cma stereo_demod.ml ../../lib/ocaml/lib-pprz.cma
	$(OCAMLC) $(INCLUDES) -I ../multimon -o $@ lablgtk.cma glibivy-ocaml.cma lib-pprz.cma multimon.cma $^
	@cat ../../../pprz_src_test.sh > $@
	@echo 'exec lablgtk2 -I $$PAPARAZZI_SRC/sw/lib/ocaml -I $$PAPARAZZI_SRC/sw/ground_segment/multimon glibivy-ocaml.cma lib-pprz.cma -I $$PAPARAZZI_SRC/sw/ground_segment/tmtc $$PAPARAZZI_SRC/sw/ground_segment/multimon/multimon.cma $$PAPARAZZI_SRC/sw/ground_segment/tmtc/stereo_demod.ml $$*' >> $@
	@chmod a+x $@

receive : receive.ml ../../lib/ocaml/lib-pprz.cma
	$(OCAMLC) $(INCLUDES) -o $@ lablgtk.cma glibivy-ocaml.cma lib-pprz.cma $^
	@cat ../../../pprz_src_test.sh > $@
	@echo 'exec lablgtk2 -I $$PAPARAZZI_SRC/sw/lib/ocaml glibivy-ocaml.cma lib-pprz.cma $$PAPARAZZI_SRC/sw/ground_segment/tmtc/receive.ml $$*' >> $@
	@chmod a+x $@


modem.cmo : modem.cmi

%.cmo : %.ml
	$(OCAMLC) $(INCLUDES) -c $<

%.cmi : %.mli
	$(OCAMLC) $(INCLUDES) $<
