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


LOCAL_CFLAGS= $(CTL_BRD_FLAGS) $(GPS_FLAGS) $(SIMUL_FLAGS)

CONF_DIR = ../../../conf
CONF_XML = $(CONF_DIR)/conf.xml

ARCH	  = atmega128
TARGET 	  = autopilot

ifeq ($(CTL_BRD_VERSION),V1_2_1)
LOW_FUSE  = e0
HIGH_FUSE = 99
CTL_BRD_FLAGS=-DCTL_BRD_V1_2_1
endif

ifeq ($(CTL_BRD_VERSION),V1_2)
LOW_FUSE  = e0
HIGH_FUSE = 99
CTL_BRD_FLAGS=-DCTL_BRD_V1_2
endif

EXT_FUSE  = ff
LOCK_FUSE = ff
INCLUDES = -I ../autopilot -I ../fly_by_wire -I ../../include
OCAMLC = ocamlc -I ../../lib/ocaml

AP=../autopilot


$(TARGET).srcs	=	\
	main.c		\
	$(AP)/modem.c	\
	$(AP)/link_fbw.c\
	$(AP)/spi.c	\
	$(AP)/adc.c	\
	$(AP)/uart.c	\
	kalman.c	\
	imu.c		\
	control.c       \


include ../../../conf/Makefile.local
include ../../../conf/Makefile.avr

autopilot.install : warn_conf

warn_conf :
	@echo
	@echo '###########################################################'
	@grep AIRFRAME_NAME ../fly_by_wire/airframe.h
	@grep RADIO_NAME ../fly_by_wire/radio.h
#	@grep FLIGHT_PLAN_NAME flight_plan.h
	@echo '###########################################################'
	@echo


#.depend : messages.h flight_plan.h  ubx_protocol.h  inflight_calib.h
main.o : messages.h
nav.o : flight_plan.h
gps_ubx.o : ubx_protocol.h
if_calib.o : inflight_calib.h

GEN_MESSAGES = ./gen_messages.out
MESSAGES_XML = $(CONF_DIR)/messages.xml

messages.h : $(MESSAGES_XML) $(GEN_MESSAGES)
	$(GEN_MESSAGES) $< > $@

$(GEN_MESSAGES) : $(AP)/gen_messages.ml
	$(OCAMLC) -o $@ xml-light.cma $<

clean : avr_clean
	rm -f *.out *.cm* messages.h flight_plan.h ubx_protocol.h inflight_calib.h
