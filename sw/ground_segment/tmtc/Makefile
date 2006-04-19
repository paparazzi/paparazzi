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

all: hw_modem_listen server messages wavecard $(VAR)/boa.conf maxstream
# stereo_demod

clean:
	rm -f hw_modem_listen wavecard maxstream server messages *.bak *~ core *.o .depend *.opt *.out *.cm*

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

messages : messages.cmo
	$(OCAMLC) -custom $(INCLUDES) -o $@ lablgtk.cma glibivy-ocaml.cma lib-pprz.cma gtkInit.cmo $^


hw_modem_listen : modem.cmo hw_modem_listen.cmo
	$(OCAMLC) -custom $(INCLUDES) -o $@ lablgtk.cma glibivy-ocaml.cma lib-pprz.cma $^

stereo_demod : ../multimon/multimon.cma stereo_demod.ml ../../lib/ocaml/lib-pprz.cma
	$(OCAMLC) $(INCLUDES) -I ../multimon -o $@ lablgtk.cma glibivy-ocaml.cma lib-pprz.cma multimon.cma $^
	@cat ../../../pprz_src_test.sh > $@
	@echo 'exec lablgtk2 -I $$PAPARAZZI_SRC/sw/lib/ocaml -I $$PAPARAZZI_SRC/sw/ground_segment/multimon glibivy-ocaml.cma lib-pprz.cma -I $$PAPARAZZI_SRC/sw/ground_segment/tmtc $$PAPARAZZI_SRC/sw/ground_segment/multimon/multimon.cma $$PAPARAZZI_SRC/sw/ground_segment/tmtc/stereo_demod.ml $$*' >> $@
	@chmod a+x $@

server : aircraft.cmo wind.cmo airprox.cmo server.cmo
	$(OCAMLC) -custom $(INCLUDES) -o $@ lablgtk.cma glibivy-ocaml.cma lib-pprz.cma $^

wavecard : wavecard_connect.cmo
	$(OCAMLC) -custom $(INCLUDES) -o $@ lablgtk.cma glibivy-ocaml.cma lib-pprz.cma $^

maxstream : maxstream.cmo ../../lib/ocaml/lib-pprz.cma
	$(OCAMLC) -custom $(INCLUDES) -o $@ lablgtk.cma glibivy-ocaml.cma lib-pprz.cma $<

dl_setting : dl_setting.cmo
	$(OCAMLC) -custom $(INCLUDES) -o $@ lablgtk.cma glibivy-ocaml.cma lib-pprz.cma gtkInit.cmo $^

modem.cmo : modem.cmi
modem.cmi :  ../../lib/ocaml/lib-pprz.cma
hw_modem_listen.cmo :  ../../lib/ocaml/lib-pprz.cma
maxstream.cmo :  ../../lib/ocaml/lib-pprz.cma

%.cmo : %.ml
	$(OCAMLC) $(INCLUDES) -c $<

%.cmi : %.mli
	$(OCAMLC) $(INCLUDES) $<

airprox.cmi aircraft.cmo : aircraft.cmi
airprox.cmo : airprox.cmi

wavecard_c : wavecard_c.c wavecard_utils_c.c wavecard_foo_c.c
	gcc -Wall -o $@ $^ `pkg-config --cflags glib-2.0` `pkg-config --libs glib-2.0`

