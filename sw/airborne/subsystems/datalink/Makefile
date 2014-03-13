#
# Copyright (C) 2013 Gautier Hattenberger
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

#
# Export downlink library source code using PPRZ transport protocol
#
# The library as to be compiled by the target application
# after implementing the dl_parse function (for parsing incoming messages)
#

# Quiet compilation
Q=@

TAR= tar
INCLUDES= -I$(PAPARAZZI_SRC)/sw/include -I$(PAPARAZZI_SRC)/sw/airborne

ifeq ($(PAPARAZZI_HOME),)
PAPARAZZI_HOME=$(PAPARAZZI_SRC)
endif

# export in $(PAPARAZZI_HOME)/var/export/datalink
EXPORT_ROOT ?= $(PAPARAZZI_HOME)/var/export
EXPORT_DIR = $(EXPORT_ROOT)/datalink

EXPORT_FILES = \
	downlink.h \
	downlink.c \
	pprz_transport.h \
	pprz_transport.c \
	transport.h \
	datalink.h \
	$(PAPARAZZI_SRC)/sw/include/std.h \
	$(PAPARAZZI_SRC)/sw/include/message_pragmas.h \
	$(PAPARAZZI_HOME)/var/include/messages.h \
	$(PAPARAZZI_HOME)/var/include/dl_protocol.h

MAKEFILE_EXPORT = pprz_datalink.mk
TAR_EXPORT = pprz_datalink.tar

all:
	@cat README.md

export_lib:
	@echo Exporting datalink lib to $(EXPORT_DIR)
	$(Q)test -d $(EXPORT_DIR) || mkdir -p $(EXPORT_DIR)
	$(Q)cp $(EXPORT_FILES) $(EXPORT_DIR)
	$(Q)cp $(MAKEFILE_EXPORT) $(EXPORT_DIR)
	$(Q)cp README.md $(EXPORT_DIR)
	$(Q)cd $(EXPORT_ROOT); $(TAR) -cf $(TAR_EXPORT) datalink/*

clean:
	$(Q)rm -rf $(EXPORT_DIR)

.PHONY: all clean export_lib

