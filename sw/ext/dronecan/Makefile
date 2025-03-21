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

PAPARAZZI_SRC=../../..
DRONECAN_DIR=$(PAPARAZZI_SRC)/sw/ext/dronecan
DSDL_DIR := $(DRONECAN_DIR)/DSDL

all: libcanard DSDL pydronecan dronecan_dsdlc

libcanard: libcanard.sync libcanard.update

DSDL : DSDL.sync DSDL.update

dronecan_dsdlc : dronecan_dsdlc.sync dronecan_dsdlc.update dronecan_dsdlc.build

pydronecan : pydronecan.sync pydronecan.update



dronecan_dsdlc.build:
	$(eval DSDL_ROOTS_PART := $(sort $(dir $(wildcard $(DSDL_DIR)/*/))))
	$(eval DSDL_ROOTS := $(filter-out $(DRONECAN_DIR)/DSDL/tests/ $(DRONECAN_DIR)/DSDL/,$(DSDL_ROOTS_PART)))
	@echo Generating dsdl code from $(DSDL_ROOTS) to $(PAPARAZZI_SRC)/var/include/DSDLcode
	$(Q) python $(DRONECAN_DIR)/dronecan_dsdlc/dronecan_dsdlc.py -O $(PAPARAZZI_SRC)/var/include/DSDLcode $(DSDL_ROOTS)
	@echo Done


# update (and init if needed) a specific submodule
%.update:
	$(Q)if [ -d $(PAPARAZZI_SRC)/.git ]; then \
		cd $(PAPARAZZI_SRC) && git submodule update --init --recursive sw/ext/dronecan/$*; \
	fi

# sync a specific submodule
%.sync:
	$(Q)if [ -d $(PAPARAZZI_SRC)/.git ]; then \
		cd $(PAPARAZZI_SRC) && git submodule sync --recursive sw/ext/dronecan/$*; \
	fi


clean:
	$(Q)rm -r $(PAPARAZZI_SRC)/var/include/DSDLcode

.PHONY: all clean pydronecan dronecan dronecan_dsdlc.build DSDL libcanard
