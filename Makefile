#   Paparazzi main $Id$
#   Copyright (C) 2004 Pascal Brisset Antoine Drouin
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

PAPARAZZI_SRC=$(shell pwd)

include conf/Makefile.local

LIB=sw/lib
AIRBORNE=sw/airborne
CONFIGURATOR=sw/configurator
COCKPIT=sw/ground_segment/cockpit
TMTC=sw/ground_segment/tmtc
MULTIMON=sw/ground_segment/multimon
WIND=sw/ground_segment/wind
VISU3D=sw/ground_segment/visu3d
LOGALIZER=sw/logalizer
SIMULATOR=sw/simulator
SUPERVISION=sw/supervision/paparazzi.pl
MAKE=make

all: static

static : lib tools configurator cockpit tmtc logalizer sim_static wind static_h multimon

conf: conf/conf.xml conf/control_panel.xml

conf/%.xml :conf/%.xml.example 
	[ -L $@ ] || [ -f $@ ] || cp $< $@ 


test: static ac1 ac2
	PAPARAZZI_HOME=$(PAPARAZZI_SRC) PAPARAZZI_SRC=$(PAPARAZZI_SRC) $(SUPERVISION)

ac1 : conf sim_static
	make AIRCRAFT=Twin1 PAPARAZZI_HOME=$(PAPARAZZI_SRC) sim_ac
ac2 : conf sim_static
	make AIRCRAFT=Twin2 PAPARAZZI_HOME=$(PAPARAZZI_SRC) sim_ac

lib:
	cd $(LIB)/ocaml; $(MAKE)
	cd $(LIB)/perl; $(MAKE)

tools: lib
	cd $(TOOLS); make

logalizer: lib
	cd $(LOGALIZER); $(MAKE)

configurator: lib
	cd $(CONFIGURATOR); $(MAKE)

sim_static :
	cd $(SIMULATOR); $(MAKE) PAPARAZZI_SRC=$(PAPARAZZI_SRC)

sim_sitl :
	cd $(SIMULATOR);  $(MAKE) sim_sitl PAPARAZZI_HOME=$(PAPARAZZI_SRC) PAPARAZZI_SRC=$(PAPARAZZI_SRC) AIRCRAFT=$(AIRCRAFT)

fbw fly_by_wire: ac_h
	cd $(AIRBORNE); $(MAKE) TARGET=fbw all

ap autopilot: ac_h
	cd $(AIRBORNE); $(MAKE) TARGET=ap all

sim: ac_h
	cd $(AIRBORNE); $(MAKE) TARGET=sim ARCHI=sim all

upload_fbw: fbw
	cd $(AIRBORNE); $(MAKE) TARGET=fbw upload

upload_ap: ap
	cd $(AIRBORNE); $(MAKE) TARGET=ap upload

erase_fbw:
	cd $(AIRBORNE); $(MAKE) TARGET=fbw erase

erase_ap:
	cd $(AIRBORNE); $(MAKE) TARGET=ap erase

airborne: fbw ap

cockpit: lib
	cd $(COCKPIT); $(MAKE) all

tmtc: lib
	cd $(TMTC); $(MAKE) all

multimon:
	cd $(MULTIMON); $(MAKE)

visu3d: lib
	cd $(VISU3D); $(MAKE)
wind:
	cd $(WIND); $(MAKE)

wr_fuses_ap:
	cd $(AIRBORNE); make TARGET=ap wr_fuses

wr_fuses_fbw:
	cd $(AIRBORNE); make TARGET=fbw wr_fuses


rd_fuses_ap:
	cd $(AIRBORNE); make TARGET=ap rd_fuses

rd_fuses_fbw:
	cd $(AIRBORNE); make TARGET=fbw rd_fuses


check_fuses_ap:
	cd $(AIRBORNE); make TARGET=ap check_fuses

check_fuses_fbw:
	cd $(AIRBORNE); make TARGET=fbw check_fuses


static_h :
	PAPARAZZI_HOME=`pwd` PAPARAZZI_SRC=`pwd` make -f Makefile.gen

ac_h : tools static_h
	PAPARAZZI_HOME=`pwd` PAPARAZZI_SRC=`pwd` $(TOOLS)/gen_aircraft.out $(AIRCRAFT)

sim_ac: ac_h sim_sitl
hard_ac: ac_h fbw ap
ac: hard_ac

doxygen:
	mkdir -p dox
	doxygen Doxyfile

clean_ac :
	rm -fr $(PAPARAZZI_HOME)/var/$(AIRCRAFT)

run_sitl :
	$(PAPARAZZI_HOME)/var/$(AIRCRAFT)/sim/simsitl

install :
	./Makefile.pl -install -destdir $(DESTDIR)

uninstall :
	./Makefile.pl -uninstall -destdir $(DESTDIR)

deb :
	chmod u+x debian/rules
	dpkg-buildpackage -rfakeroot

clean:
	rm -fr dox
	find . -mindepth 2 -name Makefile -exec sh -c '$(MAKE) -C `dirname {}` $@' \; 
	find . -name '*~' -exec rm -f {} \;

dist_clean : clean



test_all_example_airframes:
	make AIRCRAFT=Tux     clean_ac hard_ac
	make AIRCRAFT=Plaster clean_ac hard_ac	
	make AIRCRAFT=Twin1   clean_ac hard_ac
	make AIRCRAFT=Twin2   clean_ac hard_ac
	make AIRCRAFT=MJ1     clean_ac hard_ac
	make AIRCRAFT=TJ1     clean_ac ap
	make AIRCRAFT=MJ4     clean_ac ap
	make AIRCRAFT=GRZE3   clean_ac ap
	make AIRCRAFT=Twin4   clean_ac hard_ac
	make AIRCRAFT=TS5     clean_ac hard_ac
