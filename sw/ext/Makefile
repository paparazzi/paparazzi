# Hey Emacs, this is a -*- makefile -*-
#
#   Copyright (C) 2012 The Paparazzi Team
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

PAPARAZZI_SRC=../..
EXT_DIR=$(PAPARAZZI_SRC)/sw/ext

#
# find compiler toolchain
#
include $(PAPARAZZI_SRC)/conf/Makefile.arm-embedded-toolchain

MY_PYTHON := $(shell echo `which python3`)
MY_MAVLINKTOOLS := $(shell $(MY_PYTHON) -c 'import imp; import future' 2>&1)
MY_DRONECANTOOLS := $(shell $(MY_PYTHON) -c 'import em' 2>&1)

all: chibios fatfs libsbp TRICAL hacl-c key_generator rustlink ecl matrix mavlink dronecan unifiedmocaprouter

# update (and init if needed) all submodules
update_submodules:
	$(Q)if [ -d $(PAPARAZZI_SRC)/.git ]; then \
		cd $(PAPARAZZI_SRC) && git submodule update --init --recursive; \
	fi

# update (and init if needed) a specific submodule
%.update:
	$(Q)if [ -d $(PAPARAZZI_SRC)/.git ]; then \
		cd $(PAPARAZZI_SRC) && git submodule update --init --recursive sw/ext/$*; \
	fi

# sync a specific submodule
%.sync:
	$(Q)if [ -d $(PAPARAZZI_SRC)/.git ]; then \
		cd $(PAPARAZZI_SRC) && git submodule sync --recursive sw/ext/$*; \
	fi

hacl-c: hacl-c.update

key_generator: key_generator.update

rustlink: rustlink.update

chibios: chibios.sync chibios.update

TRICAL: TRICAL.update

fatfs: fatfs.sync fatfs.update

ecl: ecl.update

matrix: matrix.update

mavlink: mavlink.update mavlink.build

mavlink.build:
ifneq ($(findstring ModuleNotFoundError, $(MY_MAVLINKTOOLS)),ModuleNotFoundError)
	@echo GENERATE $(PAPARAZZI_SRC)/var/include/mavlink
	$(Q)PYTHONPATH=$(EXT_DIR)/mavlink python $(EXT_DIR)/mavlink/pymavlink/tools/mavgen.py --output $(PAPARAZZI_SRC)/var/include/mavlink --lang C $(EXT_DIR)/mavlink/message_definitions/v1.0/ardupilotmega.xml --wire-protocol 2.0 --no-validate > /dev/null
	$(Q)PYTHONPATH=$(EXT_DIR)/mavlink python $(EXT_DIR)/mavlink/pymavlink/tools/mavgen.py --output $(PAPARAZZI_SRC)/var/include/mavlink --lang C $(EXT_DIR)/mavlink/message_definitions/v1.0/paparazzi.xml --wire-protocol 2.0 --no-validate > /dev/null
else
	@echo mavlink not installed, missing packages 'imp' or 'future'
endif

libsbp: libsbp.update

pprzlink: pprzlink.update pprzlink.build

pprzlink.build:
	$(Q)$(MAKE) -C pprzlink

opencv_bebop: opencv_bebop.update opencv_bebop.build

opencv_bebop.build:
	$(Q)$(MAKE) -C opencv_bebop

dronecan:
ifneq ($(findstring ModuleNotFoundError, $(MY_DRONECANTOOLS)),ModuleNotFoundError)
	$(Q)$(MAKE) -C dronecan
else
	@echo dronecan not installed, missing package 'em'
endif

unifiedmocaprouter: unifiedmocaprouter.update unifiedmocaprouter.build

unifiedmocaprouter.build:
	cd $(EXT_DIR)/unifiedmocaprouter/ && mkdir -p build && cd build && cmake -D'MOCAPS=optitrack' -D'AGENTS=console;ivy' .. && make


clean:


clean_opencv_bebop:
	$(Q)if [ -f opencv_bebop/Makefile ]; then \
		$(MAKE) -C opencv_bebop clean; \
	fi

.NOTPARALLEL: 
.PHONY: all clean update_submodules \
		chibios fatfs \
		mavlink.build libsbp pprzlink pprzlink.build opencv_bebop \
		opencv_bebop.build clean_opencv_bebop unifiedmocaprouter TRICAL dronecan
