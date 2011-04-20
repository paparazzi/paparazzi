# Hey Emacs, this is a -*- makefile -*-
#
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
  $(error No fucking spaces allowed in the current directory name)
endif
ifeq ($(PAPARAZZI_HOME),)
PAPARAZZI_HOME=$(PAPARAZZI_SRC)
endif

LIB=sw/lib
AIRBORNE=sw/airborne
COCKPIT=sw/ground_segment/cockpit
TMTC=sw/ground_segment/tmtc
MULTIMON=sw/ground_segment/multimon
MISC=sw/ground_segment/misc
LOGALIZER=sw/logalizer
SIMULATOR=sw/simulator
MAKE=make PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME)
CONF=$(PAPARAZZI_SRC)/conf
STATICINCLUDE =$(PAPARAZZI_HOME)/var/include
MESSAGES_H=$(STATICINCLUDE)/messages.h
MESSAGES2_H=$(STATICINCLUDE)/messages2.h
UBX_PROTOCOL_H=$(STATICINCLUDE)/ubx_protocol.h
XSENS_PROTOCOL_H=$(STATICINCLUDE)/xsens_protocol.h
DL_PROTOCOL_H=$(STATICINCLUDE)/dl_protocol.h
DL_PROTOCOL2_H=$(STATICINCLUDE)/dl_protocol2.h
MESSAGES_XML = $(CONF)/messages.xml
UBX_XML = $(CONF)/ubx.xml
XSENS_XML = $(CONF)/xsens_MTi-G.xml
TOOLS=$(PAPARAZZI_SRC)/sw/tools
HAVE_ARM_NONE_EABI_GCC := $(shell which arm-none-eabi-gcc)
ifeq ($(strip $(HAVE_ARM_NONE_EABI_GCC)),)
ARMGCC=$(shell which arm-elf-gcc)
else
ARMGCC=$(HAVE_ARM_NONE_EABI_GCC)
endif
OCAML=$(shell which ocaml)
OCAMLRUN=$(shell which ocamlrun)

all: commands static conf

static : lib center tools cockpit multimon tmtc misc logalizer lpc21iap sim_static static_h usb_lib

conf: conf/conf.xml conf/control_panel.xml

conf/%.xml :conf/%.xml.example
	[ -L $@ ] || [ -f $@ ] || cp $< $@


lib:
	cd $(LIB)/ocaml; $(MAKE)

center: lib
	cd sw/supervision; make

tools: lib
	cd $(TOOLS); make

logalizer: lib
	cd $(LOGALIZER); $(MAKE)

sim_static : lib
	cd $(SIMULATOR); $(MAKE) PAPARAZZI_SRC=$(PAPARAZZI_SRC)

cockpit: lib
	cd $(COCKPIT); $(MAKE) all

tmtc: lib cockpit
	cd $(TMTC); $(MAKE) all

misc:
	cd $(MISC); $(MAKE) all

multimon:
	cd $(MULTIMON); $(MAKE)

static_h: $(MESSAGES_H) $(MESSAGES2_H) $(UBX_PROTOCOL_H) $(XSENS_PROTOCOL_H) $(DL_PROTOCOL_H) $(DL_PROTOCOL2_H)

usb_lib:
	@[ -d sw/airborne/arch/lpc21/lpcusb ] && ((test -x "$(ARMGCC)" && (cd sw/airborne/arch/lpc21/lpcusb; $(MAKE))) || echo "Not building usb_lib: ARMGCC=$(ARMGCC) not found") || echo "Not building usb_lib: sw/airborne/arch/lpc21/lpcusb directory missing"

$(MESSAGES_H) : $(MESSAGES_XML) $(CONF_XML) tools
	$(Q)test -d $(STATICINCLUDE) || mkdir -p $(STATICINCLUDE)
	@echo BUILD $@
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) $(TOOLS)/gen_messages.out $< telemetry > /tmp/msg.h
	$(Q)mv /tmp/msg.h $@
	$(Q)chmod a+r $@

$(MESSAGES2_H) : $(MESSAGES_XML) $(CONF_XML) tools
	$(Q)test -d $(STATICINCLUDE) || mkdir -p $(STATICINCLUDE)
	@echo BUILD $@
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) $(TOOLS)/gen_messages2.out $< telemetry > /tmp/msg2.h
	$(Q)mv /tmp/msg2.h $@
	$(Q)chmod a+r $@

$(UBX_PROTOCOL_H) : $(UBX_XML) tools
	@echo BUILD $@
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) $(TOOLS)/gen_ubx.out $< > /tmp/ubx.h
	$(Q)mv /tmp/ubx.h $@

$(XSENS_PROTOCOL_H) : $(XSENS_XML) tools
	@echo BUILD $@
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) $(TOOLS)/gen_xsens.out $< > /tmp/xsens.h
	$(Q)mv /tmp/xsens.h $@

$(DL_PROTOCOL_H) : $(MESSAGES_XML) tools
	@echo BUILD $@
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) $(TOOLS)/gen_messages.out $< datalink > /tmp/dl.h
	$(Q)mv /tmp/dl.h $@

$(DL_PROTOCOL2_H) : $(MESSAGES_XML) tools
	@echo BUILD $@
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) $(TOOLS)/gen_messages2.out $< datalink > /tmp/dl2.h
	$(Q)mv /tmp/dl2.h $@

include Makefile.ac

sim : sim_static


ac_h ac1 ac2 ac3 ac fbw ap: static conf

##### preliminary hard wired arm7 bootloader rules
#
#
# call with : make bl PROC=[TINY|FBW|AP|GENERIC]
bl:
	cd $(AIRBORNE)/arch/lpc21/test/bootloader; make clean; make

BOOTLOADER_DEV=/dev/ttyUSB0
upload_bl bl.upload: bl
	lpc21isp -control $(AIRBORNE)/arch/lpc21/test/bootloader/bl.hex $(BOOTLOADER_DEV) 38400 12000

JTAG_INTERFACE = olimex-jtag-tiny.cfg
#JTAG_INTERFACE = olimex-arm-usb-ocd.cfg

upload_jtag: bl
	openocd -f interface/$(JTAG_INTERFACE)  -f board/olimex_lpc_h2148.cfg -c init -c halt -c "flash write_image erase $(AIRBORNE)/arch/lpc21/test/bootloader/bl.hex"  -c reset -c shutdown



lpc21iap:
	cd sw/ground_segment/lpc21iap; make

upgrade_bl bl.upgrade: bl lpc21iap
	$(PAPARAZZI_SRC)/sw/ground_segment/lpc21iap/lpc21iap $(AIRBORNE)/arch/lpc21/test/bootloader/bl_ram.elf
	$(PAPARAZZI_SRC)/sw/ground_segment/lpc21iap/lpc21iap $(AIRBORNE)/arch/lpc21/test/bootloader/bl.elf

ms:
	cd $(AIRBORNE)/arch/lpc21/lpcusb; make
	cd $(AIRBORNE)/arch/lpc21/lpcusb/examples; make

upload_ms ms.upload: ms
	$(PAPARAZZI_SRC)/sw/ground_segment/lpc21iap/lpc21iap $(AIRBORNE)/arch/lpc21/lpcusb/examples/msc.elf

#####
#####

doxygen:
	mkdir -p dox
	doxygen Doxyfile

run_sitl :
	$(PAPARAZZI_HOME)/var/$(AIRCRAFT)/sim/simsitl

install :
	make -f Makefile.install PREFIX=$(PREFIX)

uninstall :
	make -f Makefile.install PREFIX=$(PREFIX) uninstall

DISTRO=lenny
deb :
	chmod u+x debian/rules
	cp debian/control.$(DISTRO) debian/control
	cp debian/changelog.$(DISTRO) debian/changelog
	dpkg-buildpackage $(DEBFLAGS) -Ivar -rfakeroot

fast_deb:
	make deb OCAMLC=ocamlc.opt DEBFLAGS=-b

clean:
	rm -fr dox build-stamp configure-stamp conf/%gconf.xml debian/files debian/paparazzi-arm7 debian/paparazzi-avr debian/paparazzi-base debian/paparazzi-bin debian/paparazzi-dev
	rm -f  $(MESSAGES_H) $(MESSAGES2_H) $(UBX_PROTOCOL_H) $(DL_PROTOCOL_H)
	find . -mindepth 2 -name Makefile -exec sh -c '$(MAKE) -C `dirname {}` $@' \;
	find . -name '*~' -exec rm -f {} \;
	rm -f paparazzi sw/simulator/launchsitl

cleanspaces:
	find ./sw/airborne -name '*.[ch]' -exec sed -i {} -e 's/[ \t]*$$//' \;
	find ./conf -name '*.makefile' -exec sed -i {} -e 's/[ \t]*$$//' ';'
	find ./sw -name '*.ml' -exec sed -i {} -e 's/[ \t]*$$//' ';'
	find ./sw -name '*.mli' -exec sed -i {} -e 's/[ \t]*$$//' ';'
	find ./conf -name '*.xml' -exec sed -i {} -e 's/[ \t]*$$//' ';'

distclean : dist_clean
dist_clean : clean
	rm -r conf/srtm_data


ab_clean:
	find sw/airborne -name '*~' -exec rm -f {} \;

test_all_example_airframes:
	$(MAKE) AIRCRAFT=BOOZ2_A1 clean_ac ap sim
	$(MAKE) AIRCRAFT=Microjet clean_ac ap sim
	$(MAKE) AIRCRAFT=Tiny_IMU clean_ac ap
	$(MAKE) AIRCRAFT=EasyStar_ETS clean_ac ap sim

commands: paparazzi sw/simulator/launchsitl

paparazzi:
	cat src/paparazzi | sed s#OCAMLRUN#$(OCAMLRUN)# | sed s#OCAML#$(OCAML)# > $@
	chmod a+x $@

sw/simulator/launchsitl:
	cat src/$(@F) | sed s#OCAMLRUN#$(OCAMLRUN)# | sed s#OCAML#$(OCAML)# > $@
	chmod a+x $@

#.SUFFIXES: .hgt.zip

%.hgt.zip:
	cd data/srtm; $(MAKE) $(@)

