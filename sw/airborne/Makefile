# Hey Emacs, this is a -*- makefile -*-
#
#  Copyright (C) 2003-2014  The Paparazzi Team
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

# Quiet compilation
Q=@


# main directory where the generated files and compilation results for an aircraft are stored
AIRCRAFT_BUILD_DIR = $(PAPARAZZI_HOME)/var/aircrafts/$(AIRCRAFT)

OBJDIR = $(AIRCRAFT_BUILD_DIR)/$(TARGET)
ACINCLUDE = $(AIRCRAFT_BUILD_DIR)/$(TARGET)
VARINCLUDE = $(PAPARAZZI_HOME)/var/include

INCLUDES = -I$(PAPARAZZI_SRC)/sw/include -I$(PAPARAZZI_SRC)/sw/airborne -I$(PAPARAZZI_SRC)/conf/autopilot -I$(PAPARAZZI_SRC)/sw/airborne/arch/$($(TARGET).ARCHDIR) -I$(VARINCLUDE) -I$(ACINCLUDE)

VPATH = .

ifneq ($(MAKECMDGOALS),clean)
  include $(AIRCRAFT_BUILD_DIR)/Makefile.ac
  $(TARGET).srcs += $($(TARGET).EXTRA_SRCS)
  include ../../conf/Makefile.local

# check if ARCHDIR is set
  ifeq ($($(TARGET).ARCHDIR), )
    $(error Architecture not set, maybe you forgot to add the target? e.g. <target name="tunnel" board="twog_1.0"/>)
  else
    ifdef $(TARGET).MAKEFILE
      include ../../conf/Makefile.$($(TARGET).MAKEFILE)
    else
      include ../../conf/Makefile.$($(TARGET).ARCHDIR)
    endif
  endif

  $(TARGET).CFLAGS += -DGIT_SHA1=$(GIT_SHA1)
  $(TARGET).CFLAGS += -DGIT_DESC=$(GIT_DESC)
  $(TARGET).CFLAGS += -DPPRZ_VER=$(PPRZ_VER)
  $(TARGET).CFLAGS += -DPPRZ_VER_MAJOR=$(PPRZ_VER_MAJOR)
  $(TARGET).CFLAGS += -DPPRZ_VER_MINOR=$(PPRZ_VER_MINOR)
  $(TARGET).CFLAGS += -DPPRZ_VER_PATCH=$(PPRZ_VER_PATCH)
  ifdef PRINT_CONFIG
    $(TARGET).CFLAGS += -DPRINT_CONFIG
  endif

  # sort cflags and sources to throw out duplicates
  #
  $(TARGET).CFLAGS := $(sort $($(TARGET).CFLAGS))
  $(TARGET).srcs := $(sort $($(TARGET).srcs))
endif


$(TARGET).install : warn_conf

warn_conf :
	@echo
	@echo '###########################################################'
	@grep AIRFRAME_NAME $(ACINCLUDE)/generated/airframe.h
	@grep RADIO_NAME $(ACINCLUDE)/generated/radio.h
	@grep FLIGHT_PLAN_NAME $(ACINCLUDE)/generated/flight_plan.h
	@echo '###########################################################'
	@echo

.PHONY: clean
