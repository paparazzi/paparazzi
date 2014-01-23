# Hey Emacs, this is a -*- makefile -*-
#
#  $Id$
#  Copyright (C) 2003-2005 Pascal Brisset, Antoine Drouin
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


OBJDIR = $(PAPARAZZI_HOME)/var/$(AIRCRAFT)/$(TARGET)
VARINCLUDE=$(PAPARAZZI_HOME)/var/include
ACINCLUDE = $(PAPARAZZI_HOME)/var/$(AIRCRAFT)

INCLUDES = -I$(PAPARAZZI_SRC)/sw/include -I$(PAPARAZZI_SRC)/sw/airborne -I$(PAPARAZZI_SRC)/conf/autopilot -I$(PAPARAZZI_SRC)/sw/airborne/arch/$($(TARGET).ARCHDIR) -I$(VARINCLUDE) -I$(ACINCLUDE)


ifneq ($(MAKECMDGOALS),clean)
  include $(PAPARAZZI_HOME)/var/$(AIRCRAFT)/Makefile.ac
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
