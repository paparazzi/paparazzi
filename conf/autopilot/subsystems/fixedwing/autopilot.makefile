#
# $Id: autopilot.makefile 4827 2010-04-21 08:02:18Z poine $
#
# Copyright (C) 2008 Antoine Drouin
#
# This file is part of paparazzi.
#tin
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

######################################################################
##
## COMMON FIXEDWING ALL TARGETS (SIM + AP + FBW ...)
##

# temporary hack for ADCs
ifeq ($(ARCH), stm32)
# FIXME : this is for the battery
$(TARGET).CFLAGS += -DUSE_AD1_3
endif
#
# Board config + Include paths
#

$(TARGET).CFLAGS 	+= -DBOARD_CONFIG=$(BOARD_CFG)
$(TARGET).CFLAGS 	+= -DPERIPHERALS_AUTO_INIT
$(TARGET).CFLAGS 	+= $(FIXEDWING_INC)

$(TARGET).srcs 	+= mcu.c
$(TARGET).srcs 	+= $(SRC_ARCH)/mcu_arch.c

#
# Common Options
#

ifeq ($(OPTIONS), minimal)
else
  $(TARGET).CFLAGS 	+= -DWIND_INFO
endif

$(TARGET).CFLAGS 	+= -DTRAFFIC_INFO

#
# LEDs
#

$(TARGET).CFLAGS 	+= -DUSE_LED
ifneq ($(ARCH), lpc21)
  ifneq ($(ARCH), jsbsim)
    $(TARGET).srcs 	+= $(SRC_ARCH)/led_hw.c
  endif
endif

#
# Sys-time
#
ifndef PERIODIC_FREQUENCY
PERIODIC_FREQUENCY = 60
endif
$(TARGET).CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./$(PERIODIC_FREQUENCY).))' -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)
$(TARGET).srcs += sys_time.c

#
# InterMCU & Commands
#

$(TARGET).CFLAGS 	+= -DINTER_MCU
$(TARGET).srcs 		+= $(SRC_FIXEDWING)/inter_mcu.c

######################################################################
##
## COMMON FOR ALL NON-SIMULATION TARGETS
##

#
# Interrupt Vectors
#

ifeq ($(ARCH), lpc21)
  ns_srcs 		+= $(SRC_ARCH)/armVIC.c
else ifeq ($(ARCH), stm32)
  ns_srcs 		+= $(SRC_ARCH)/stm32_exceptions.c
  ns_srcs 		+= $(SRC_ARCH)/stm32_vector_table.c
#  ns_CFLAGS 		+= -DPERIPHERALS_AUTO_INIT
endif

ifeq ($(ARCH), stm32)
  ns_srcs 		+= lisa/plug_sys.c
endif


#
# Main
#

ns_srcs	   	+= $(SRC_FIRMWARE)/main.c

#
# LEDs
#

ns_CFLAGS 		+= -DUSE_LED
ifeq ($(ARCH), stm32)
  ns_CFLAGS 	+= -DSYS_TIME_LED=1
else
  ns_CFLAGS 	+= -DTIME_LED=1
endif

#
# Sys-time
#

ns_CFLAGS 		+= -DUSE_SYS_TIME
ns_srcs 		+= $(SRC_ARCH)/sys_time_hw.c


#
# UARTS
#

ns_srcs 		+= $(SRC_ARCH)/mcu_periph/uart_arch.c
ns_srcs 		+= subsystems/settings.c
ns_srcs 		+= $(SRC_ARCH)/subsystems/settings_arch.c

#
# ANALOG
#

  ns_CFLAGS 		+= -DUSE_ADC
#ifeq ($(ARCH), lpc21)
  ns_srcs 		+= $(SRC_ARCH)/mcu_periph/adc_arch.c
ifeq ($(ARCH), stm32)
  ns_CFLAGS 		+= -DUSE_AD1 -DUSE_AD1_1 -DUSE_AD1_2 -DUSE_AD1_3 -DUSE_AD1_4
  ns_CFLAGS 		+= -DUSE_ADC1_2_IRQ_HANDLER
endif

######################################################################
##
## FLY BY WIRE THREAD SPECIFIC
##

fbw_CFLAGS		+= -DFBW
fbw_srcs 		+= $(SRC_FIRMWARE)/main_fbw.c
fbw_srcs 		+= subsystems/electrical.c
fbw_srcs 		+= $(SRC_FIXEDWING)/commands.c

######################################################################
##
## AUTOPILOT THREAD SPECIFIC
##

ap_CFLAGS 		+= -DAP
ap_srcs 		+= $(SRC_FIRMWARE)/main_ap.c
ap_srcs 		+= $(SRC_FIXEDWING)/estimator.c


######################################################################
##
## SIMULATOR THREAD SPECIFIC
##

UNAME = $(shell uname -s)
ifeq ("$(UNAME)","Darwin")
  sim.CFLAGS += -I/opt/local/include/
endif

sim.CFLAGS              += $(CPPFLAGS)
sim.CFLAGS 		+= $(fbw_CFLAGS) $(ap_CFLAGS)
sim.srcs 		+= $(fbw_srcs) $(ap_srcs)

sim.CFLAGS 		+= -DSITL
sim.srcs 		+= $(SRC_ARCH)/sim_ap.c

sim.CFLAGS 		+= -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport
sim.srcs 		+= downlink.c $(SRC_FIRMWARE)/datalink.c $(SRC_ARCH)/sim_gps.c $(SRC_ARCH)/ivy_transport.c $(SRC_ARCH)/sim_adc_generic.c

sim.srcs 		+= subsystems/settings.c
sim.srcs 		+= $(SRC_ARCH)/subsystems/settings_arch.c

######################################################################
##
## JSBSIM THREAD SPECIFIC
##

OCAMLLIBDIR=$(shell ocamlc -where)
JSBSIM_INC = /usr/include/JSBSim
#JSBSIM_LIB = /usr/lib

jsbsim.CFLAGS 		+= $(fbw_CFLAGS) $(ap_CFLAGS)
jsbsim.srcs 		+= $(fbw_srcs) $(ap_srcs)

jsbsim.CFLAGS 		+= -DSITL
jsbsim.srcs 		+= $(SIMDIR)/sim_ac_jsbsim.c $(SIMDIR)/sim_ac_fw.c $(SIMDIR)/sim_ac_flightgear.c

# external libraries
jsbsim.CFLAGS 		+= -I$(SIMDIR) -I/usr/include -I$(JSBSIM_INC) -I$(OCAMLLIBDIR) `pkg-config glib-2.0 --cflags`
jsbsim.LDFLAGS		+= `pkg-config glib-2.0 --libs` -lm -lpcre -lglibivy -L/usr/lib -lJSBSim

jsbsim.CFLAGS 		+= -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport
jsbsim.srcs 		+= downlink.c $(SRC_FIRMWARE)/datalink.c $(SRC_ARCH)/jsbsim_hw.c $(SRC_ARCH)/jsbsim_gps.c $(SRC_ARCH)/ivy_transport.c $(SRC_ARCH)/jsbsim_transport.c

######################################################################
##
## Final Target Allocations
##

#
# SINGLE MCU / DUAL MCU
#

ifeq ($(BOARD),classix)
  fbw.CFLAGS 		+= -DMCU_SPI_LINK -DUSE_SPI -DSPI_SLAVE
  fbw.srcs 		+= $(SRC_FIXEDWING)/link_mcu.c mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c
  ap.CFLAGS 		+= -DMCU_SPI_LINK -DUSE_SPI -DSPI_MASTER -DUSE_SPI_SLAVE0
  ap.srcs 		+= $(SRC_FIXEDWING)/link_mcu.c mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c
else
  # Single MCU's run both
  ap.CFLAGS 		+= $(fbw_CFLAGS)
  ap.srcs 		+= $(fbw_srcs)
endif

#
# No-Sim parameters
#

fbw.CFLAGS 		+= $(fbw_CFLAGS) $(ns_CFLAGS)
fbw.srcs 		+= $(fbw_srcs) $(ns_srcs)

ap.CFLAGS 		+= $(ap_CFLAGS) $(ns_CFLAGS)
ap.srcs 		+= $(ap_srcs) $(ns_srcs)
