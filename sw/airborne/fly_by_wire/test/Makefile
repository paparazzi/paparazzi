# 
#  $Id$
#  Copyright (C) 2005 Pascal Brisset, Antoine Drouin
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


all:
	@echo "call with 'make TARGET=... compile (or load)'"

LOCAL_CFLAGS= $(CTL_BRD_FLAGS)

ARCH      = atmega8
TARGET	  = check_uart


VARINCLUDE = $(PAPARAZZI_HOME)/var/include
ACINCLUDE = $(PAPARAZZI_HOME)/var/$(AIRCRAFT)
INCLUDES = -I ../ -I ../../../include -I $(ACINCLUDE) -I $(VARINCLUDE)
#-I ../../../var/include

CONF_DIR = ../../../../conf
CONF_XML = $(CONF_DIR)/conf.xml
XML_GET=../../../lib/ocaml/xml_get.out

ifeq ($(CTL_BRD_VERSION),V1_2_1)
CTL_BRD_FLAGS=-DCTL_BRD_V1_2_1
endif

ifeq ($(CTL_BRD_VERSION),V1_2)
CTL_BRD_FLAGS=-DCTL_BRD_V1_2
endif

ifeq ($(CTL_BRD_VERSION),V1_1)
CTL_BRD_FLAGS=-DCTL_BRD_V1_1
endif

rc_transmitter.srcs  = rc_transmitter.c ../ppm.c ../uart.c

new_rc_transmitter.srcs  = new_rc_transmitter.c ../ppm.c ../uart.c ../telemetry.c

setup_servos.srcs  = setup_servos.c ../uart.c ../servo.c

check_uart.srcs  = check_uart.c ../uart.c

tx_adcs.srcs = tx_adcs.c ../uart.c ../adc_fbw.c

check_spi.srcs = check_spi.c ../uart.c ../spi.c


include ../../../../conf/Makefile.local
include ../../../../conf/Makefile.avr

main.o .depend : $(ACINCLUDE)/radio.h $(ACINCLUDE)/airframe.h

clean: avr_clean
