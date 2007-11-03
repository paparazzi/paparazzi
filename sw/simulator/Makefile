# Paparazzi simulator $Id$
#  
# Copyright (C) 2003-2006  Pascal Brisset, Antoine Drouin
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

# Quiet compilation
Q=@

include ../../conf/Makefile.local

SIMML = stdlib.ml data.ml flightModel.ml gps.ml
SIMHML = $(SIMML) hitl.ml sim.ml
SIMHCMO=$(SIMHML:%.ml=%.cmo)
SIMSML = $(SIMML) sitl.ml sim.ml
SIMSCMO=$(SIMSML:%.ml=%.cmo)
SIMSCMX=$(SIMSML:%.ml=%.cmx)


OCAMLC = ocamlc -g
OCAMLOPT=ocamlopt
INCLUDES= -I +lablgtk2 -I ../lib/ocaml -I +xml-light

AIRBORNE = ../airborne
VARINCLUDE=$(PAPARAZZI_HOME)/var/include
ACINCLUDE = $(PAPARAZZI_HOME)/var/$(AIRCRAFT)
LAUNCHSITL = launchsitl

all : gaia sitl.cma simhitl $(LAUNCHSITL)
	@chmod a+x $(LAUNCHSITL)

simhitl : fg.o $(SIMHCMO) simhitl.cmo
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ str.cma xml-light.cma unix.cma glibivy-ocaml.cma lib-pprz.cma lablgtk.cma gtkInit.cmo $^

sitl.cma : fg.o $(SIMSCMO)
	@echo OL $@
	$(Q)ocamlmklib -o sitl $^ 

sitl.cmxa : $(SIMSCMX)
	ocamlopt -o $@ -a $^ 

gaia : gaia.cmo
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma glibivy-ocaml.cma lib-pprz.cma lablgtk.cma gtkInit.cmo $<

diffusion : stdlib.cmo diffusion.cmo
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -o $@ unix.cma str.cma xml-light.cma glibivy-ocaml.cma lib-pprz.cma lablgtk.cma gtkInit.cmo $^

%.cmo : %.ml ../lib/ocaml/lib-pprz.cma
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

%.o : %.c
	@echo OC $<
	$(Q)$(OCAMLC) -c $<

%.cmx : %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(INCLUDES) -c $<

%.cmi : %.mli ../lib/ocaml/lib-pprz.cma
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

clean :
	rm -f *.cm* *~ *.out .depend *.o *.a *.so gaia simhitl booz_sim

#
# Dependencies
#

.depend:
	ocamldep *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif

AB=../airborne
AB_ARCH=$(AB)/sim

CC = gcc
CFLAGS= -g -Wall `pkg-config glib-2.0 --cflags` -I /usr/include/meschach -I$(AB) -I../include -I../../var/BOOZ -I../../var/include -I../../conf/autopilot -I$(AB_ARCH)
LDFLAGS=`pkg-config glib-2.0 --libs` -lm -lmeschach -lpcre


BOOZ_SIM_SRCS = main_booz_sim.c           \
	        booz_flight_model.c       \
                booz_flight_model_utils.c \
                booz_sensors_model.c      \
                booz_flightgear.c         \
                booz_joystick.c           \

CFLAGS += -DSITL
CFLAGS += -DBOOZ_CONTROLLER_MCU
CFLAGS += -DCONFIG=\"conf_booz.h\"

BOOZ_AB_SRCS += $(AB)/booz_controller_main.c

BOOZ_AB_SRCS += $(AB)/sys_time.c

CFLAGS += -DRADIO_CONTROL 
BOOZ_AB_SRCS += $(AB)/radio_control.c \
		$(AB_ARCH)/ppm_hw.c

CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" 
BOOZ_AB_SRCS += $(AB)/actuators.c \
                $(AB_ARCH)/actuators_buss_twi_blmc_hw.c \
                $(AB)/i2c.c $(AB_ARCH)/i2c_hw.c

CFLAGS += -DDOWNLINK 
BOOZ_AB_SRCS += $(AB)/booz_controller_telemetry.c \
                $(AB)/downlink.c

#CFLAGS += -DDOWNLINK_TRANSPORT=PprzTransport -DPPRZ_UART=SimUart -DDOWNLINK_DEVICE=SimUart -DSIM_UART
#BOOZ_AB_SRCS += ../airborne/pprz_transport.c  ../airborne/sim/sim_uart.c 

CFLAGS += -DDOWNLINK_TRANSPORT=IvyTransport
BOOZ_AB_SRCS += $(AB_ARCH)/ivy_transport.c
LDFLAGS += -lglibivy

BOOZ_AB_SRCS += $(AB)/booz_inter_mcu.c
CFLAGS += -DUSE_SPI
BOOZ_AB_SRCS += $(AB)/booz_link_mcu.c $(AB_ARCH)/booz_link_mcu_hw.c
BOOZ_AB_SRCS += $(AB)/spi.c $(AB_ARCH)/spi_hw.c

#CFLAGS += -DDATALINK=PPRZ
#BOOZ_AB_SRCS += ../airborne/datalink.c
BOOZ_AB_SRCS += $(AB)/booz_estimator.c
BOOZ_AB_SRCS += $(AB)/booz_control.c
BOOZ_AB_SRCS += $(AB)/booz_nav.c
BOOZ_AB_SRCS += $(AB)/booz_nav_hover.c

BOOZ_AB_SRCS += $(AB)/booz_autopilot.c
BOOZ_AB_SRCS += $(AB)/commands.c


CFLAGS += -DBOOZ_FILTER_MCU
BOOZ_AB_SRCS += $(AB)/booz_filter_main.c

CFLAGS += -DADC_CHANNEL_AX=1 -DADC_CHANNEL_AY=2 -DADC_CHANNEL_AZ=3 -DADC_CHANNEL_BAT=4
BOOZ_AB_SRCS  += $(AB_ARCH)/adc_hw.c


BOOZ_AB_SRCS += $(AB)/booz_filter_telemetry.c

BOOZ_AB_SRCS  += $(AB)/max1167.c $(AB_ARCH)/max1167_hw.c
BOOZ_AB_SRCS  += $(AB)/micromag.c $(AB_ARCH)/micromag_hw.c
BOOZ_AB_SRCS  += $(AB)/imu_v3.c $(AB_ARCH)/imu_v3_hw.c


BOOZ_AB_SRCS += $(AB)/booz_ahrs.c
CFLAGS += -DBOOZ_AHRS_TYPE=BOOZ_AHRS_MULTITILT
BOOZ_AB_SRCS += $(AB)/multitilt.c
#CFLAGS += -DBOOZ_AHRS_TYPE=BOOZ_AHRS_QUATERNION -DEKF_UPDATE_DISCRETE
#BOOZ_AB_SRCS += $(AB)/ahrs_quat_fast_ekf.c 

#$(OBJDIR)/%.c: %.o
#	$(CC) $(CFLAGS) -c -o $@ $<

booz_sim: $(BOOZ_SIM_SRCS) $(BOOZ_AB_SRCS)
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)
