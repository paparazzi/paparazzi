# Paparazzi simulator $Id$
#  
# Copied from autopilot (autopilot.sf.net) thanx alot Trammell
#
# Copyright (C) 2003-2006  Pascal Brisset, Antoine Drouin
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

include ../../conf/Makefile.local

ACDIR= $(PAPARAZZI_HOME)/var/$(AIRCRAFT)

SIMHML = stdlib.ml data.ml flightModel.ml gps.ml hitl.ml sim.ml
SIMHCMO=$(SIMHML:%.ml=%.cmo)
SIMSML = stdlib.ml data.ml flightModel.ml gps.ml sitl.ml sim.ml
SIMSCMO=$(SIMSML:%.ml=%.cmo)
SIMSCMX=$(SIMSML:%.ml=%.cmx)

#SIMSC = sim_ir.c sim_gps.c sim_ap.c sys_time.c estimator.c infrared.c gps.c pid.c nav.c radio_control.c main_ap.c cam.c traffic_info.c
#SIMSO=$(SIMSC:%.c=$(OBJDIR)/%.o)
#SIMSA=sims.cma
SIMSO = $(sim.srcs:.c=$(OBJDIR)/%.o)

OCAMLC = ocamlc
OCAMLOPT=ocamlopt -p
INCLUDES= -I +lablgtk2 -I ../lib/ocaml
OCAMLCC = gcc -Werror -W -Wall -O2 -I /usr/include/glib-2.0 -I /usr/lib/glib-2.0/include -DSITL $(ap.CFLAGS) -I $(OBJDIR) -I $(ACDIR) 

AIRBORNE = ../airborne
VARINCLUDE=$(PAPARAZZI_HOME)/var/include
ACINCLUDE = $(PAPARAZZI_HOME)/var/$(AIRCRAFT)


SIMDIR=$(shell echo `pwd`)



all : gaia sitl.cma fg.o

sim_sitl : $(OBJDIR)/simsitl

simhitl.out : $(SIMHCMO) simhitl.cmo
	$(OCAMLC) $(INCLUDES) -o $@ str.cma xml-light.cma unix.cma lib.cma lablgtk.cma gtkInit.cmo $^

sitl.cma : fg.o $(SIMSCMO)
	ocamlmklib -o sitl $^ 

fg.o : fg.c
	$(OCAMLC) -c fg.c

sitl.cmxa : $(SIMSCMX)
	ocamlopt -o $@ -a $^ 

#$(OBJDIR)/$(SIMSA) : $(SIMSO)
#	@touch $(OBJDIR)/dummy.ml
#	@cd $(OBJDIR); ocamlmklib -o sims $^ dummy.ml



gaia : gaia.cmo
	$(OCAMLC) -custom $(INCLUDES) -o $@ glibivy-ocaml.cma lib-pprz.cma lablgtk.cma gtkInit.cmo $<

$(OBJDIR)/%.o : %.c
	$(OCAMLCC) -c -o $@ -I $(SIMDIR) -I $(AIRBORNE) -I $(AIRBORNE) -I ../include -I $(VARINCLUDE) $<

$(OBJDIR)/%.o : $(AIRBORNE)/%.c
	$(OCAMLCC) -c -o $@ -I $(SIMDIR) -I $(AIRBORNE) -I $(AIRBORNE) -I ../include -I $(VARINCLUDE) $<

$(OBJDIR)/main_ap.o : $(OBJDIR)/main_ap.c
	$(OCAMLCC) -c -o $@ -I $(SIMDIR) -I $(AIRBORNE) -I $(AIRBORNE) -I ../include -I $(VARINCLUDE) $<

$(OBJDIR)/sim_gps.o $(OBJDIR)/nav.o $(OBJDIR)/main_ap.o $(OBJDIR)/sim_ir.o $(OBJDIR)/sim_ap.o $(OBJDIR)/pid.o $(OBJDIR)/estimator.o $(OBJDIR)/cam.o $(OBJDIR)/gps.o : $(OBJDIR)/sitl_messages.h  $(ACINCLUDE)/flight_plan.h  $(ACINCLUDE)/airframe.h

$(OBJDIR)/sim_ap.o : $(AIRBORNE)/traffic_info.h $(AIRBORNE)/nav.h 

$(OBJDIR)/main_ap.c : $(OBJDIR)/sitl_messages.h $(AIRBORNE)/main_ap.c
	cp $(AIRBORNE)/main_ap.c $(@)

$(OBJDIR)/simsitl.cmo : $(OBJDIR)/simsitl.ml
	$(OCAMLC) $(INCLUDES) -c -o $@ $<

$(OBJDIR)/simsitl.cmx : $(OBJDIR)/simsitl.ml
	$(OCAMLOPT) $(INCLUDES) -c -o $@ $<

%.cmo : %.ml
	$(OCAMLC) $(INCLUDES) -c $<

%.o : %.c
	$(OCAMLC) -c $<

%.cmx : %.ml
	$(OCAMLOPT) $(INCLUDES) -c $<

%.cmi : %.mli
	$(OCAMLC) $(INCLUDES) -c $<

clean :
	\rm -f *.cm* *~ *.out .depend *.o *.a *.so gaia

#
# Dependencies
#

.depend:
	ocamldep *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif
