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

include conf/Makefile.local

LIB=sw/lib
AIRBORNE=sw/airborne
CONFIGURATOR=sw/configurator
FBW=$(AIRBORNE)/fly_by_wire
AP=$(AIRBORNE)/autopilot
COCKPIT=sw/ground_segment/cockpit
TMTC=sw/ground_segment/tmtc
WIND=sw/ground_segment/wind
VISU3D=sw/ground_segment/visu3d
LOGALIZER=sw/logalizer
SIMULATOR=sw/simulator
MAKE=make

static : lib tools configurator cockpit tmtc visu3d logalizer sim_static wind 

configure : configurator
	PAPARAZZI_DIR=`pwd` $(CONFIGURATOR)/configurator

lib:
	cd $(LIB)/ocaml; $(MAKE)
	cd $(LIB)/perl; $(MAKE)

tools:
	cd $(TOOLS); make

logalizer: lib
	cd $(LOGALIZER); $(MAKE)

configurator: lib
	cd $(CONFIGURATOR); $(MAKE)

sim_static :
	cd $(SIMULATOR); $(MAKE)

sim_sitl :
	cd $(SIMULATOR); $(MAKE) sim_sitl

fbw fly_by_wire:
	cd $(FBW); $(MAKE) all

ap autopilot:
	cd $(AP); $(MAKE) all

upload_fbw: fbw
	cd $(FBW); $(MAKE) upload

upload_ap: ap
	cd $(AP); $(MAKE) upload

erase_fbw:
	cd $(FBW); $(MAKE) erase

erase_ap:
	cd $(AP); $(MAKE) erase

airborne: fbw ap

cockpit: lib
	cd $(COCKPIT); $(MAKE) all

tmtc: lib
	cd $(TMTC); $(MAKE) all

visu3d: lib
	cd $(VISU3D); $(MAKE)
wind:
	cd $(WIND); $(MAKE)

receive: tmtc
	$(TMTC)/receive

static_h :
	make -f Makefile.gen

ac_h :
	$(TOOLS)/gen_aircraft.out $(AIRCRAFT)

ac: static_h ac_h ap fbw sim_sitl

clean_ac :
	rm -fr $(PAPARAZZI_HOME)/var/$(AIRCRAFT)

run_sitl :
	$(PAPARAZZI_HOME)/var/$(AIRCRAFT)/sim/simsitl.out

t1: ac

install : static t1
	./Makefile.pl -install -destdir $(DESTDIR)

uninstall :
	./Makefile.pl -uninstall -destdir $(DESTDIR)

deb :
	dpkg-buildpackage -rfakeroot

clean:
	find . -name Makefile -mindepth 2 -exec sh -c '$(MAKE) -C `dirname {}` $@' \; 
	find . -name '*~' -exec rm -f {} \;

