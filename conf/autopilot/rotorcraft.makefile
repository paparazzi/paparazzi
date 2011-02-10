# Hey Emacs, this is a -*- makefile -*-
#
# $Id$
#
# Copyright (C) 2010 The Paparazzi Team
#
# This file is part of Paparazzi.
#
# Paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# Paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#
#

CFG_SHARED=$(PAPARAZZI_SRC)/conf/autopilot/subsystems/shared
CFG_ROTORCRAFT=$(PAPARAZZI_SRC)/conf/autopilot/subsystems/rotorcraft

SRC_BOOZ=booz
SRC_BOOZ_ARCH=$(SRC_BOOZ)/arch/$(ARCH)
SRC_BOOZ_TEST=$(SRC_BOOZ)/test
SRC_BOOZ_PRIV=booz_priv

SRC_BOARD=boards/$(BOARD)
SRC_FIRMWARE=firmwares/rotorcraft
SRC_SUBSYSTEMS=subsystems

SRC_ARCH=arch/$(ARCH)

CFG_BOOZ=$(PAPARAZZI_SRC)/conf/autopilot/

ROTORCRAFT_INC = -I$(SRC_FIRMWARE) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -I$(SRC_BOARD)


ap.ARCHDIR = $(ARCH)


ap.CFLAGS += $(ROTORCRAFT_INC)
ap.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -DPERIPHERALS_AUTO_INIT
ap.srcs    = $(SRC_FIRMWARE)/main.c
ap.srcs   += mcu.c
ap.srcs   += $(SRC_ARCH)/mcu_arch.c

ifeq ($(ARCH), stm32)
ap.srcs += lisa/plug_sys.c
endif
#
# Interrupts
#
ifeq ($(ARCH), lpc21)
ap.srcs += $(SRC_ARCH)/armVIC.c
else ifeq ($(ARCH), stm32)
ap.srcs += $(SRC_ARCH)/stm32_exceptions.c
ap.srcs += $(SRC_ARCH)/stm32_vector_table.c
endif

#
# LEDs
#
ap.CFLAGS += -DUSE_LED
ifeq ($(ARCH), stm32)
ap.srcs += $(SRC_ARCH)/led_hw.c
endif

# frequency of main periodic
ifndef PERIODIC_FREQUENCY
PERIODIC_FREQUENCY = 512
endif
$(TARGET).CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./$(PERIODIC_FREQUENCY).))' -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)
#
# Systime
#
ap.CFLAGS += -DUSE_SYS_TIME
ap.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
ifeq ($(ARCH), stm32)
ap.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif

#
# Telemetry/Datalink
#
# include subsystems/rotorcraft/telemetry_transparent.makefile
# or
# include subsystems/rotorcraft/telemetry_xbee_api.makefile
#
ap.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

# I2C is needed for speed controllers and barometers on lisa
ap.srcs += mcu_periph/i2c.c
ap.srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.c

ap.srcs += $(SRC_BOOZ)/booz2_commands.c

#
# Radio control choice
#
# include subsystems/rotorcraft/radio_control_ppm.makefile
# or
# include subsystems/rotorcraft/radio_control_spektrum.makefile
#

#
# Actuator choice
#
# include subsystems/rotorcraft/actuators_mkk.makefile
# or
# include subsystems/rotorcraft/actuators_asctec.makefile
# or
# include subsystems/rotorcraft/actuators_asctec_v2.makefile
#

#
# IMU choice
#
# include subsystems/rotorcraft/imu_b2v1.makefile
# or
# include subsystems/rotorcraft/imu_b2v1_1.makefile
# or
# include subsystems/rotorcraft/imu_crista.makefile
#

#
# BARO
#
ap.srcs += $(SRC_BOARD)/baro_board.c
ifeq ($(BOARD), booz)
ap.CFLAGS += -DROTORCRAFT_BARO_LED=$(BARO_LED) -DBOOZ2_ANALOG_BARO_PERIOD='SYS_TICS_OF_SEC((1./100.))'
else ifeq ($(BOARD), lisa_l)
ap.CFLAGS += -DUSE_I2C2
endif

#
# Analog Backend
#
ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DBOOZ2_ANALOG_BATTERY_PERIOD='SYS_TICS_OF_SEC((1./10.))'
ap.srcs += $(SRC_FIRMWARE)/battery.c
ap.CFLAGS += -DADC0_VIC_SLOT=2
ap.CFLAGS += -DADC1_VIC_SLOT=3
ap.srcs += $(SRC_BOOZ)/booz2_analog.c \
		   $(SRC_BOOZ_ARCH)/booz2_analog_hw.c
else ifeq ($(ARCH), stm32)
ap.srcs += lisa/lisa_analog_plug.c
endif


#
# GPS choice
#
# include subsystems/rotorcraft/gps_ubx.makefile
# or
# include subsystems/rotorcraft/gps_skytraq.makefile
# or
# nothing
#


#
# AHRS choice
#
# include subsystems/rotorcraft/ahrs_cmpl.makefile
# or
# include subsystems/rotorcraft/ahrs_lkf.makefile
#

ap.srcs += $(SRC_FIRMWARE)/autopilot.c

ap.srcs += math/pprz_trig_int.c
ap.srcs += $(SRC_FIRMWARE)/stabilization.c
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_rate.c


ap.CFLAGS += -DSTABILISATION_ATTITUDE_TYPE_INT
ap.CFLAGS += -DSTABILISATION_ATTITUDE_H=\"stabilization/stabilization_attitude_int.h\"
ap.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/stabilization_attitude_ref_euler_int.h\"
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_euler_int.c
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_euler_int.c

ap.CFLAGS += -DUSE_NAVIGATION
ap.srcs += $(SRC_FIRMWARE)/guidance/guidance_h.c
ap.srcs += $(SRC_FIRMWARE)/guidance/guidance_v.c

ap.srcs += $(SRC_SUBSYSTEMS)/ins.c
ap.srcs += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c

#
# INS choice
#
# include subsystems/rotorcraft/ins_hff.makefile
# or
# nothing
#

#  vertical filter float version
ap.srcs += $(SRC_SUBSYSTEMS)/ins/vf_float.c
ap.CFLAGS += -DUSE_VFF -DDT_VFILTER='(1./$(PERIODIC_FREQUENCY).)'

ap.srcs += $(SRC_FIRMWARE)/navigation.c


#
# FMS  choice
#
# include booz2_fms_test_signal.makefile
# or
# include booz2_fms_datalink.makefile
# or
# nothing
#
