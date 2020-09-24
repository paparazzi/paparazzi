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

PAPARAZZI_SRC ?= $(shell pwd)
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
STATICINCLUDE=$(PAPARAZZI_HOME)/var/include
CONF=$(PAPARAZZI_SRC)/conf
AIRBORNE=sw/airborne
SIMULATOR=sw/simulator
COCKPIT=sw/ground_segment/cockpit
TMTC=sw/ground_segment/tmtc
GENERATORS=$(PAPARAZZI_SRC)/sw/tools/generators
JOYSTICK=sw/ground_segment/joystick
EXT=sw/ext
TOOLS=sw/tools

#
# build some stuff in subdirs
# nothing should depend on these...
#
PPRZCENTER=sw/supervision
MISC=sw/ground_segment/misc
LOGALIZER=sw/logalizer

SUBDIRS = $(PPRZCENTER) $(MISC) $(LOGALIZER) sw/tools

#
# Communication protocol version
#
PPRZLINK_LIB_VERSION ?= 2.0

#
# xml files used as input for header generation
#
CUSTOM_MESSAGES_XML = $(CONF)/messages.xml
ABI_XML = $(CONF)/abi.xml
UBX_XML = $(CONF)/ubx.xml
MTK_XML = $(CONF)/mtk.xml
XSENS_XML = $(CONF)/xsens_MTi-G.xml

#
# generated header files
#
PPRZLINK_DIR=sw/ext/pprzlink
PPRZLINK_INSTALL=$(PAPARAZZI_HOME)/var/lib
MESSAGES_INSTALL=$(PAPARAZZI_HOME)/var
UBX_PROTOCOL_H=$(STATICINCLUDE)/ubx_protocol.h
MTK_PROTOCOL_H=$(STATICINCLUDE)/mtk_protocol.h
XSENS_PROTOCOL_H=$(STATICINCLUDE)/xsens_protocol.h
ABI_MESSAGES_H=$(STATICINCLUDE)/abi_messages.h
MAVLINK_DIR=$(STATICINCLUDE)/mavlink/
MAVLINK_PROTOCOL_H=$(MAVLINK_DIR)protocol.h

GEN_HEADERS = $(UBX_PROTOCOL_H) $(MTK_PROTOCOL_H) $(XSENS_PROTOCOL_H) $(ABI_MESSAGES_H) $(MAVLINK_PROTOCOL_H)

all: ground_segment ext lpctools

_print_building:
	@echo "------------------------------------------------------------"
	@echo "Building Paparazzi version" $(shell ./paparazzi_version)
	@echo "------------------------------------------------------------"

print_build_version:
	@echo "------------------------------------------------------------"
	@echo "Last build Paparazzi version" $(shell cat $(PAPARAZZI_HOME)/var/build_version.txt 2> /dev/null || echo UNKNOWN)
	@echo "------------------------------------------------------------"

_save_build_version:
	$(Q)test -d $(PAPARAZZI_HOME)/var || mkdir -p $(PAPARAZZI_HOME)/var
	$(Q)./paparazzi_version > $(PAPARAZZI_HOME)/var/build_version.txt

update_google_version:
	-$(MAKE) -C data/maps

init:
	@[ -d $(PAPARAZZI_HOME) ] || (echo "Copying config example in your $(PAPARAZZI_HOME) directory"; mkdir -p $(PAPARAZZI_HOME); cp -a conf $(PAPARAZZI_HOME); cp -a data $(PAPARAZZI_HOME); mkdir -p $(PAPARAZZI_HOME)/var/maps; mkdir -p $(PAPARAZZI_HOME)/var/include)

conf: conf/conf.xml conf/control_panel.xml conf/maps.xml conf/tools/blacklisted

conf/%.xml :conf/%_example.xml
	[ -L $@ ] || [ -f $@ ] || cp $< $@

conf/tools/blacklisted: conf/tools/blacklisted_example
	cp conf/tools/blacklisted_example conf/tools/blacklisted

ground_segment: _print_building update_google_version conf libpprz subdirs static
ground_segment.opt: ground_segment cockpit.opt tmtc.opt

static: cockpit tmtc generators sim_static joystick static_h

libpprzlink.update:
	$(MAKE) -C $(EXT) pprzlink.update

libpprzlink.install:
	$(Q)Q=$(Q) MAKEFLAGS=-j1 DESTDIR=$(PPRZLINK_INSTALL) PPRZLINK_LIB_VERSION=${PPRZLINK_LIB_VERSION} $(MAKE) -C $(PPRZLINK_DIR) libpprzlink-install

libpprz: libpprzlink.update libpprzlink.install _save_build_version
	$(MAKE) -C $(LIB)/ocaml

cockpit: libpprz
	$(MAKE) -C $(COCKPIT)

cockpit.opt: libpprz
	$(MAKE) -C $(COCKPIT) opt

tmtc: libpprz cockpit
	$(MAKE) -C $(TMTC)

tmtc.opt: libpprz cockpit.opt
	$(MAKE) -C $(TMTC) opt

generators: libpprz
	$(MAKE) -C $(GENERATORS)

joystick: libpprz
	$(MAKE) -C $(JOYSTICK)

sim_static: libpprz
	$(MAKE) -C $(SIMULATOR)

ext:
	$(MAKE) -C $(EXT)
	$(MAKE) -C $(TOOLS)/bluegiga_usb_dongle

opencv_bebop:
	$(MAKE) -C $(EXT) opencv_bebop

#
# make misc subdirs
#
subdirs: $(SUBDIRS)

$(MISC): ext

$(SUBDIRS): libpprz
	$(MAKE) -C $@

$(PPRZCENTER): libpprz

$(LOGALIZER): libpprz

static_h: pprzlink_protocol $(GEN_HEADERS)

pprzlink_protocol : libpprz
	$(Q)test -d $(STATICINCLUDE) || mkdir -p $(STATICINCLUDE)
	$(Q)test -d $(STATICLIB) || mkdir -p $(STATICLIB)
ifeq ("$(wildcard $(CUSTOM_MESSAGES_XML))","")
	@echo GENERATE $@ with default messages
	$(Q)Q=$(Q) MESSAGES_INSTALL=$(MESSAGES_INSTALL) VALIDATE_XML=FALSE PPRZLINK_LIB_VERSION=${PPRZLINK_LIB_VERSION} $(MAKE) -C $(PPRZLINK_DIR) pymessages
else
	@echo GENERATE $@ with custome messages from $(CUSTOM_MESSAGES_XML)
	$(Q)Q=$(Q) MESSAGES_XML=$(CUSTOM_MESSAGES_XML) MESSAGES_INSTALL=$(MESSAGES_INSTALL) PPRZLINK_LIB_VERSION=${PPRZLINK_LIB_VERSION} $(MAKE) -C $(PPRZLINK_DIR) pymessages
endif


$(UBX_PROTOCOL_H) : $(UBX_XML) generators
	@echo GENERATE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME) $(GENERATORS)/gen_ubx.out $< > $($@_TMP)
	$(Q)mv $($@_TMP) $@
	$(Q)chmod a+r $@

$(MTK_PROTOCOL_H) : $(MTK_XML) generators
	@echo GENERATE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME) $(GENERATORS)/gen_mtk.out $< > $($@_TMP)
	$(Q)mv $($@_TMP) $@
	$(Q)chmod a+r $@

$(XSENS_PROTOCOL_H) : $(XSENS_XML) generators
	@echo GENERATE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME) $(GENERATORS)/gen_xsens.out $< > $($@_TMP)
	$(Q)mv $($@_TMP) $@
	$(Q)chmod a+r $@

$(ABI_MESSAGES_H) : $(ABI_XML) generators
	@echo GENERATE $@
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME) $(GENERATORS)/gen_abi.out $< airborne > $($@_TMP)
	$(Q)mv $($@_TMP) $@
	$(Q)chmod a+r $@

$(MAVLINK_PROTOCOL_H) :
	$(Q)make -C $(PAPARAZZI_HOME)/sw/ext mavlink

#
# code generation for aircrafts from xml files
#
include Makefile.ac

ac_h ac fbw ap: static conf generators ext

sim: sim_static


# stuff to build and upload the lpc bootloader ...
include Makefile.lpctools
lpctools: lpc21iap


#
# doxygen html documentation
#
dox:
	$(Q)PAPARAZZI_HOME=$(PAPARAZZI_HOME) sw/tools/doxygen_gen/gen_modules_doc.py -pv
	$(Q)PAPARAZZI_HOME=$(PAPARAZZI_HOME) sw/tools/doxygen_gen/gen_messages_doc.py -pv
	@echo "Generationg doxygen html documentation in doc/generated/html"
	$(Q)( cat Doxyfile ; echo "PROJECT_NUMBER=$(./paparazzi_version)"; echo "QUIET=YES") | doxygen -
	@echo "Done. Open doc/generated/html/index.html in your browser to view it."

#
# Cleaning
#

clean:
	$(Q)rm -fr dox build-stamp configure-stamp conf/%gconf.xml
	$(Q)rm -f  $(GEN_HEADERS)
	$(Q)MESSAGES_INSTALL=$(MESSAGES_INSTALL) $(MAKE) -C $(PPRZLINK_DIR) uninstall
	$(Q)rm -fr $(MAVLINK_DIR)
	$(Q)find . -mindepth 2 -name Makefile -a ! -path "./sw/ext/*" -exec sh -c 'echo "Cleaning {}"; $(MAKE) -C `dirname {}` $@' \;
	$(Q)$(MAKE) -C $(EXT) clean
	$(Q)find . -name '*~' -exec rm -f {} \;
	$(Q)find . -name '*.pyc' -exec rm -f {} \;

cleanspaces:
	find sw -path sw/ext -prune -o -type f -name '*.[ch]' -exec sed -i {} -e 's/[ \t]*$$//' \;
	find sw -path sw/ext -prune -o -type f -name '*.py' -exec sed -i {} -e 's/[ \t]*$$//' \;
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
test: test_math test_examples

# subset of airframes for coverity test to pass the limited build time on travis
test_coverity: all
	CONF_XML=conf/conf_tests_coverity.xml prove tests/aircrafts/

# test AggieAir conf
test_aggieair: all
	CONF_XML=conf/airframes/AGGIEAIR/aggieair_conf.xml prove tests/aircrafts/
	
# test Open UAS conf
test_openuas: all
	CONF_XML=conf/userconf/OPENUAS/openuas_conf.xml prove tests/aircrafts/
	
# test TU Delft conf
test_tudelft: all
	CONF_XML=conf/userconf/tudelft/conf.xml prove tests/aircrafts/

# compiles all aircrafts in conf_tests.xml
test_examples: all
	CONF_XML=conf/conf_tests.xml prove tests/aircrafts/

test_all_confs: all opencv_bebop
	$(Q)$(eval $CONFS:=$(shell ./find_confs.py))
	@echo "************\nFound $(words $($CONFS)) config files: $($CONFS)"
	$(Q)$(foreach conf,$($CONFS),echo "\n************\nTesting all aircrafts in conf: $(conf)\n************" && (CONF_XML=$(conf) prove tests/aircrafts/ || echo "failed $(conf)" >> TEST_ALL_CONFS_FAILED);) test -f TEST_ALL_CONFS_FAILED && cat TEST_ALL_CONFS_FAILED && rm -f TEST_ALL_CONFS_FAILED && exit 1; exit 0

# run some math tests that don't need whole paparazzi to be built
test_math:
	make -C tests/math

# super simple simulator test, needs X
# always uses conf/conf.xml, so that needs to contain the appropriate aircrafts
# (only Microjet right now)
test_sim: all
	prove tests/sim

.PHONY: all print_build_version _print_building _save_build_version update_google_version init dox ground_segment ground_segment.opt \
subdirs $(SUBDIRS) conf ext libpprz libpprzlink.update libpprzlink.install cockpit cockpit.opt tmtc tmtc.opt generators\
static sim_static lpctools opencv_bebop\
clean cleanspaces ab_clean dist_clean distclean dist_clean_irreversible \
test test_examples test_math test_sim test_all_confs
