# Hey Emacs, this is a -*- makefile -*-
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

# would be better to auto-generate this
$(TARGET).CFLAGS 	+= -DFIRMWARE=FIXEDWING

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
ifneq ($(ARCH), jsbsim)
  $(TARGET).CFLAGS 	+= -DUSE_LED
endif
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
$(TARGET).CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)
$(TARGET).srcs   += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c
$(TARGET).CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_RESOLUTION='(1./$(PERIODIC_FREQUENCY).)'

#
# InterMCU & Commands
#
$(TARGET).CFLAGS 	+= -DINTER_MCU
$(TARGET).srcs 		+= $(SRC_FIXEDWING)/inter_mcu.c

#
# Math functions
#
$(TARGET).srcs += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c math/pprz_trig_int.c math/pprz_orientation_conversion.c

#
# I2C
#
include $(CFG_SHARED)/i2c_select.makefile

######################################################################
##
## COMMON FOR ALL NON-SIMULATION TARGETS
##

#
# Interrupt Vectors
#
ifeq ($(ARCH), lpc21)
  ns_srcs 		+= $(SRC_ARCH)/armVIC.c
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
ifneq ($(SYS_TIME_LED),none)
  ns_CFLAGS 	+= -DSYS_TIME_LED=$(SYS_TIME_LED)
endif


#
# UARTS
#
ns_srcs 		+= mcu_periph/uart.c
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
fbw_srcs		+= $(SRC_FIRMWARE)/fbw_downlink.c

######################################################################
##
## AUTOPILOT THREAD SPECIFIC
##

ap_CFLAGS 		+= -DAP
ap_srcs 		+= $(SRC_FIRMWARE)/main_ap.c
ap_srcs			+= $(SRC_FIRMWARE)/ap_downlink.c
ap_srcs 		+= state.c

# BARO
ifeq ($(BOARD), umarim)
ifeq ($(BOARD_VERSION), 1.0)
ap_srcs 	+= boards/umarim/baro_board.c
ap_CFLAGS += -DUSE_I2C1 -DUSE_ADS1114_1
ap_CFLAGS += -DADS1114_I2C_DEVICE=i2c1
ap_srcs 	+= peripherals/ads1114.c
endif
else ifeq ($(BOARD), lisa_l)
ap_CFLAGS += -DUSE_I2C2
endif


######################################################################
##
## SIMULATOR THREAD SPECIFIC
##

UNAME = $(shell uname -s)
ifeq ("$(UNAME)","Darwin")
  sim.CFLAGS += $(shell if test -d /opt/paparazzi/include; then echo "-I/opt/paparazzi/include"; elif test -d /opt/local/include; then echo "-I/opt/local/include"; fi)
endif

sim.CFLAGS              += $(CPPFLAGS)
sim.CFLAGS 		+= $(fbw_CFLAGS) $(ap_CFLAGS)
sim.srcs 		+= $(fbw_srcs) $(ap_srcs)

sim.CFLAGS 		+= -DSITL
sim.srcs 		+= $(SRC_ARCH)/sim_ap.c

sim.CFLAGS 		+= -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport
sim.srcs 		+= subsystems/datalink/downlink.c $(SRC_FIRMWARE)/datalink.c $(SRC_ARCH)/sim_gps.c $(SRC_ARCH)/ivy_transport.c $(SRC_ARCH)/sim_adc_generic.c

sim.srcs 		+= subsystems/settings.c
sim.srcs 		+= $(SRC_ARCH)/subsystems/settings_arch.c

# hack: always compile some of the sim functions, so ocaml sim does not complain about no-existing functions
sim.srcs        += $(SRC_ARCH)/sim_ahrs.c $(SRC_ARCH)/sim_ir.c

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
jsbsim.LDFLAGS		+= `pkg-config glib-2.0 --libs` -lglibivy -lm -L/usr/lib -lJSBSim

jsbsim.CFLAGS 		+= -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport
jsbsim.srcs 		+= subsystems/datalink/downlink.c $(SRC_FIRMWARE)/datalink.c $(SRC_ARCH)/jsbsim_hw.c $(SRC_ARCH)/jsbsim_ir.c $(SRC_ARCH)/jsbsim_gps.c $(SRC_ARCH)/jsbsim_ahrs.c $(SRC_ARCH)/ivy_transport.c $(SRC_ARCH)/jsbsim_transport.c

jsbsim.srcs 		+= subsystems/settings.c
jsbsim.srcs 		+= $(SRC_ARCH)/subsystems/settings_arch.c

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
  ap_srcs		+= $(SRC_FIRMWARE)/fbw_downlink.c
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
