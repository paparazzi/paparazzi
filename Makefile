# Hey Emacs, this is a -*- makefile -*-
#
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

# The default is to produce a quiet echo of compilation commands
# Launch with "make Q=''" to get full echo
Q=@

ifeq ($(Q),@)
MAKEFLAGS += --no-print-directory
endif

PAPARAZZI_SRC=$(shell pwd)
empty=
space=$(empty) $(empty)
ifneq ($(findstring $(space),$(PAPARAZZI_SRC)),)
  $(error No spaces allowed in the current directory name)
endif
ifeq ($(PAPARAZZI_HOME),)
PAPARAZZI_HOME=$(PAPARAZZI_SRC)
endif

# export the PAPARAZZI environment to sub-make
export PAPARAZZI_SRC
export PAPARAZZI_HOME

OCAML=$(shell which ocaml)
OCAMLRUN=$(shell which ocamlrun)
BUILD_DATETIME:=$(shell date +%Y%m%d-%H%M%S)

# default mktemp in OS X doesn't work, use gmktemp with macports coreutils
UNAME = $(shell uname -s)
ifeq ("$(UNAME)","Darwin")
	MKTEMP = gmktemp
else
	MKTEMP = mktemp
endif

#
# define some paths
#
LIB=sw/lib
STATICINCLUDE =$(PAPARAZZI_HOME)/var/include
CONF=$(PAPARAZZI_SRC)/conf
AIRBORNE=sw/airborne
SIMULATOR=sw/simulator
MULTIMON=sw/ground_segment/multimon
COCKPIT=sw/ground_segment/cockpit
TMTC=sw/ground_segment/tmtc
TOOLS=$(PAPARAZZI_SRC)/sw/tools
JOYSTICK=sw/ground_segment/joystick
EXT=sw/ext

#
# build some stuff in subdirs
# nothing should depend on these...
#
PPRZCENTER=sw/supervision
MISC=sw/ground_segment/misc
LOGALIZER=sw/logalizer

SUBDIRS = $(PPRZCENTER) $(MISC) $(LOGALIZER)

#
# xml files used as input for header generation
#
MESSAGES_XML = $(CONF)/messages.xml
UBX_XML = $(CONF)/ubx.xml
MTK_XML = $(CONF)/mtk.xml
XSENS_XML = $(CONF)/xsens_MTi-G.xml

#
# generated header files
#
MESSAGES_H=$(STATICINCLUDE)/messages.h
MESSAGES2_H=$(STATICINCLUDE)/messages2.h
UBX_PROTOCOL_H=$(STATICINCLUDE)/ubx_protocol.h
MTK_PROTOCOL_H=$(STATICINCLUDE)/mtk_protocol.h
XSENS_PROTOCOL_H=$(STATICINCLUDE)/xsens_protocol.h
DL_PROTOCOL_H=$(STATICINCLUDE)/dl_protocol.h
DL_PROTOCOL2_H=$(STATICINCLUDE)/dl_protocol2.h
ABI_MESSAGES_H=$(STATICINCLUDE)/abi_messages.h

GEN_HEADERS = $(MESSAGES_H) $(MESSAGES2_H) $(UBX_PROTOCOL_H) $(MTK_PROTOCOL_H) $(XSENS_PROTOCOL_H) $(DL_PROTOCOL_H) $(DL_PROTOCOL2_H) $(ABI_MESSAGES_H)


all: ground_segment ext lpctools

print_build_version:
	@echo "------------------------------------------------------------"
	@echo "Building Paparazzi version" $(shell ./paparazzi_version)
	@echo "------------------------------------------------------------"

update_google_version:
	-$(MAKE) -C data/maps

conf: conf/conf.xml conf/control_panel.xml conf/maps.xml

conf/%.xml :conf/%_example.xml
	[ -L $@ ] || [ -f $@ ] || cp $< $@


ground_segment: print_build_version update_google_version conf libpprz subdirs commands static
ground_segment.opt: ground_segment cockpit.opt tmtc.opt

static: cockpit tmtc tools sim_static joystick static_h

libpprz:
	$(MAKE) -C $(LIB)/ocaml

multimon:
	$(MAKE) -C $(MULTIMON)

cockpit: libpprz
	$(MAKE) -C $(COCKPIT)

cockpit.opt: libpprz
	$(MAKE) -C $(COCKPIT) opt

tmtc: libpprz cockpit multimon
	$(MAKE) -C $(TMTC)

tmtc.opt: libpprz cockpit.opt multimon
	$(MAKE) -C $(TMTC) opt

tools: libpprz
	$(MAKE) -C $(TOOLS)

joystick: libpprz
	$(MAKE) -C $(JOYSTICK)

sim_static: libpprz
	$(MAKE) -C $(SIMULATOR)

ext:
	$(MAKE) -C $(EXT)

#
# make misc subdirs
#
subdirs: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

$(PPRZCENTER): libpprz

$(LOGALIZER): libpprz


static_h: $(GEN_HEADERS)

$(MESSAGES_H) : $(MESSAGES_XML) tools
	$(Q)test -d $(STATICINCLUDE) || mkdir -p $(STATICINCLUDE)
	@echo GENERATE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME) $(TOOLS)/gen_messages.out $< telemetry > $($@_TMP)
	$(Q)mv $($@_TMP) $@
	$(Q)chmod a+r $@

$(MESSAGES2_H) : $(MESSAGES_XML) tools
	$(Q)test -d $(STATICINCLUDE) || mkdir -p $(STATICINCLUDE)
	@echo GENERATE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME) $(TOOLS)/gen_messages2.out $< telemetry > $($@_TMP)
	$(Q)mv $($@_TMP) $@
	$(Q)chmod a+r $@

$(UBX_PROTOCOL_H) : $(UBX_XML) tools
	@echo GENERATE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME) $(TOOLS)/gen_ubx.out $< > $($@_TMP)
	$(Q)mv $($@_TMP) $@
	$(Q)chmod a+r $@

$(MTK_PROTOCOL_H) : $(MTK_XML) tools
	@echo GENERATE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME) $(TOOLS)/gen_mtk.out $< > $($@_TMP)
	$(Q)mv $($@_TMP) $@
	$(Q)chmod a+r $@

$(XSENS_PROTOCOL_H) : $(XSENS_XML) tools
	@echo GENERATE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME) $(TOOLS)/gen_xsens.out $< > $($@_TMP)
	$(Q)mv $($@_TMP) $@
	$(Q)chmod a+r $@

$(DL_PROTOCOL_H) : $(MESSAGES_XML) tools
	@echo GENERATE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME) $(TOOLS)/gen_messages.out $< datalink > $($@_TMP)
	$(Q)mv $($@_TMP) $@
	$(Q)chmod a+r $@

$(DL_PROTOCOL2_H) : $(MESSAGES_XML) tools
	@echo GENERATE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME) $(TOOLS)/gen_messages2.out $< datalink > $($@_TMP)
	$(Q)mv $($@_TMP) $@
	$(Q)chmod a+r $@

$(ABI_MESSAGES_H) : $(MESSAGES_XML) tools
	@echo GENERATE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME) $(TOOLS)/gen_abi.out $< airborne > $($@_TMP)
	$(Q)mv $($@_TMP) $@
	$(Q)chmod a+r $@


include Makefile.ac

ac_h ac fbw ap: static conf tools ext

sim: sim_static


#
# Commands
#

# stuff to build and upload the lpc bootloader ...
include Makefile.lpctools
lpctools: lpc21iap

commands: paparazzi

paparazzi:
	cat src/paparazzi | sed s#OCAMLRUN#$(OCAMLRUN)# | sed s#OCAML#$(OCAML)# > $@
	chmod a+x $@


#
# doxygen html documentation
#
dox:
	$(Q)PAPARAZZI_HOME=$(PAPARAZZI_HOME) sw/tools/doxygen_gen/gen_modules_doc.py -pv
	@echo "Generationg doxygen html documentation in doc/generated/html"
	$(Q)( cat Doxyfile ; echo "PROJECT_NUMBER=$(./paparazzi_version)"; echo "QUIET=YES") | doxygen -
	@echo "Done. Open doc/generated/html/index.html in your browser to view it."

#
# Cleaning
#

clean:
	$(Q)rm -fr dox build-stamp configure-stamp conf/%gconf.xml
	$(Q)rm -f  $(GEN_HEADERS)
	$(Q)find . -mindepth 2 -name Makefile -a ! -path "./sw/ext/*" -exec sh -c 'echo "Cleaning {}"; $(MAKE) -C `dirname {}` $@' \;
	$(Q)$(MAKE) -C $(EXT) clean
	$(Q)find . -name '*~' -exec rm -f {} \;

cleanspaces:
	find sw -path sw/ext -prune -o -name '*.[ch]' -exec sed -i {} -e 's/[ \t]*$$//' \;
	find conf -name '*.makefile' -exec sed -i {} -e 's/[ \t]*$$//' ';'
	find . -path ./sw/ext -prune -o -name Makefile -exec sed -i {} -e 's/[ \t]*$$//' ';'
	find sw -name '*.ml' -o -name '*.mli' -exec sed -i {} -e 's/[ \t]*$$//' ';'
	find conf -name '*.xml' -exec sed -i {} -e 's/[ \t]*$$//' ';'

distclean : dist_clean
dist_clean :
	@echo "Warning: This removes all non-repository files. This means you will loose your aircraft list, your maps, your logfiles, ... if you want this, then run: make dist_clean_irreversible"

dist_clean_irreversible: clean
	rm -rf conf/maps_data conf/maps.xml
	rm -rf conf/conf.xml conf/controlpanel.xml
	rm -rf var

ab_clean:
	find sw/airborne -name '*~' -exec rm -f {} \;


#
# Tests
#

replace_current_conf_xml:
	test conf/conf.xml && mv conf/conf.xml conf/conf.xml.backup.$(BUILD_DATETIME)
	cp conf/conf_tests.xml conf/conf.xml

restore_conf_xml:
	test conf/conf.xml.backup.$(BUILD_DATETIME) && mv conf/conf.xml.backup.$(BUILD_DATETIME) conf/conf.xml

run_tests:
	cd tests; $(MAKE) test

test: all replace_current_conf_xml run_tests restore_conf_xml


.PHONY: all print_build_version update_google_version dox ground_segment ground_segment.opt \
subdirs $(SUBDIRS) conf ext libpprz multimon cockpit cockpit.opt tmtc tmtc.opt tools\
static sim_static lpctools commands \
clean cleanspaces ab_clean dist_clean distclean dist_clean_irreversible \
test replace_current_conf_xml run_tests restore_conf_xml
