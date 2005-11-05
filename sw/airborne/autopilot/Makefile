# 
#  $Id$
#  Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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


FBW=../fly_by_wire


LOCAL_CFLAGS= $(CTL_BRD_FLAGS) $(GPS_FLAGS) $(SIMUL_FLAGS)

VARINCLUDE=$(PAPARAZZI_HOME)/var/include
ACINCLUDE = $(PAPARAZZI_HOME)/var/$(AIRCRAFT)

ARCH	  = atmega128
TARGET 	  = autopilot
TARGETDIR = autopilot

LOW_FUSE  = e0
HIGH_FUSE = 99

ifeq ($(SIMUL),1)
SIMUL_FLAGS= -DSIMUL
endif

EXT_FUSE  = ff
LOCK_FUSE = ff
INCLUDES = -I $(FBW) -I ../../include -I $(VARINCLUDE) -I $(ACINCLUDE)

GPS = gps_ubx.c
GPS_FLAGS=-DUBX

$(TARGET).srcs	= \
	main.c \
	modem.c \
	link_fbw.c \
	spi.c \
	adc.c \
	$(GPS) \
	infrared.c \
	pid.c \
	nav.c \
	uart.c \
    estimator.c \
	if_calib.c \
	mainloop.c \
	cam.c \
	traffic_info.c \
	datalink.c

ifeq ($(AIRCRAFT), Gorazoptere_brushless_ANALOG)
	$(TARGET).srcs  += ahrs.S 
	$(TARGET).srcs  += ahrs.c 
endif
ifeq ($(AIRCRAFT), quadRotorJP)
	$(TARGET).srcs  += ahrs.S 
	$(TARGET).srcs  += ahrs.c
#	$(TARGET).srcs  += fp.c
endif


include ../../../conf/Makefile.local
include ../../../conf/Makefile.avr

autopilot.install : warn_conf

warn_conf :
	@echo
	@echo '###########################################################'
	@grep AIRFRAME_NAME $(ACINCLUDE)/airframe.h
	@grep RADIO_NAME $(ACINCLUDE)/radio.h
	@grep FLIGHT_PLAN_NAME $(ACINCLUDE)/flight_plan.h
	@echo '###########################################################'
	@echo


wavecard_glib : wavecard_glib.c wavecard.c wavecard_glib_utils.c
	gcc -Wall -o $@ $^ `pkg-config --cflags glib-2.0` `pkg-config --libs glib-2.0`


clean : avr_clean
	rm -f *.out *.cm* messages.h flight_plan.h ubx_protocol.h inflight_calib.h wavecard_glib
