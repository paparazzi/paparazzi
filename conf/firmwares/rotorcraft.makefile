# Hey Emacs, this is a -*- makefile -*-
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

CFG_SHARED=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/shared
CFG_ROTORCRAFT=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/rotorcraft

SRC_BOOZ_TEST=$(SRC_BOOZ)/test
SRC_BOOZ_PRIV=booz_priv

SRC_BOARD=boards/$(BOARD)
SRC_FIRMWARE=firmwares/rotorcraft
SRC_SUBSYSTEMS=subsystems

SRC_ARCH=arch/$(ARCH)

CFG_BOOZ=$(PAPARAZZI_SRC)/conf/firmwares/

ROTORCRAFT_INC = -I$(SRC_FIRMWARE) -I$(SRC_BOARD)


ap.ARCHDIR = $(ARCH)

# would be better to auto-generate this
$(TARGET).CFLAGS 	+= -DFIRMWARE=ROTORCRAFT

ap.CFLAGS += $(ROTORCRAFT_INC)
ap.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -DPERIPHERALS_AUTO_INIT
ap.srcs    = $(SRC_FIRMWARE)/main.c
ap.srcs   += mcu.c
ap.srcs   += $(SRC_ARCH)/mcu_arch.c

#
# Math functions
#
ap.srcs += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c math/pprz_trig_int.c math/pprz_orientation_conversion.c

ifeq ($(ARCH), stm32)
ap.srcs += lisa/plug_sys.c
endif
#
# Interrupts
#
ifeq ($(ARCH), lpc21)
ap.srcs += $(SRC_ARCH)/armVIC.c
endif

#
# LEDs
#
ap.CFLAGS += -DUSE_LED
ifeq ($(ARCH), stm32)
ap.srcs += $(SRC_ARCH)/led_hw.c
endif

ifeq ($(BOARD)$(BOARD_TYPE), ardroneraw)
ap.srcs   += $(SRC_BOARD)/gpio.c
endif

# frequency of main periodic
PERIODIC_FREQUENCY ?= 512
ap.CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)

TELEMETRY_FREQUENCY ?= 60
ap.CFLAGS += -DTELEMETRY_FREQUENCY=$(TELEMETRY_FREQUENCY)

#
# Systime
#
ap.CFLAGS += -DUSE_SYS_TIME
ap.srcs += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c
ifneq ($(SYS_TIME_LED),none)
ap.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif

#
# Telemetry/Datalink
#
# include subsystems/rotorcraft/telemetry_transparent.makefile
# or
# include subsystems/rotorcraft/telemetry_xbee_api.makefile
#
ap.srcs += subsystems/settings.c
ap.srcs += $(SRC_ARCH)/subsystems/settings_arch.c

ap.srcs += mcu_periph/uart.c
ap.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

# I2C is needed for speed controllers and barometers on lisa
ifeq ($(TARGET), ap)
$(TARGET).srcs += mcu_periph/i2c.c
$(TARGET).srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.c
endif

ap.srcs += subsystems/commands.c
ap.srcs += subsystems/actuators.c

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
# booz baro
ifeq ($(BOARD), booz)
ap.srcs += $(SRC_BOARD)/baro_board.c
else ifeq ($(BOARD), lisa_l)
ap.CFLAGS += -DUSE_I2C2
ap.srcs += $(SRC_BOARD)/baro_board.c

# Ardrone baro
else ifeq ($(BOARD)$(BOARD_TYPE), ardroneraw)
ap.srcs += $(SRC_BOARD)/baro_board.c
else ifeq ($(BOARD)$(BOARD_TYPE), ardronesdk)
ap.srcs += $(SRC_BOARD)/baro_board_dummy.c

# Lisa/M baro
else ifeq ($(BOARD), lisa_m)
# defaults to i2c baro bmp085 on the board
LISA_M_BARO ?= BARO_BOARD_BMP085
  ifeq ($(LISA_M_BARO), BARO_MS5611_SPI)
    include $(CFG_SHARED)/spi_master.makefile
    ap.CFLAGS += -DUSE_SPI2 -DUSE_SPI_SLAVE3
    ap.srcs += $(SRC_BOARD)/baro_ms5611_spi.c
  else ifeq ($(LISA_M_BARO), BARO_MS5611_I2C)
    ap.CFLAGS += -DUSE_I2C2
    ap.srcs += $(SRC_BOARD)/baro_ms5611_i2c.c
  else ifeq ($(LISA_M_BARO), BARO_BOARD_BMP085)
    ap.srcs += $(SRC_BOARD)/baro_board.c
	ap.CFLAGS += -DUSE_I2C2
  endif
  ap.CFLAGS += -D$(LISA_M_BARO)

# Lia baro (no bmp onboard)
else ifeq ($(BOARD), lia)
# fixme, reuse the baro drivers in lisa_m dir
LIA_BARO ?= BARO_MS5611_SPI
  ifeq ($(LIA_BARO), BARO_MS5611_SPI)
    include $(CFG_SHARED)/spi_master.makefile
    ap.CFLAGS += -DUSE_SPI2 -DUSE_SPI_SLAVE3
    ap.srcs += boards/lisa_m/baro_ms5611_spi.c
  else ifeq ($(LIA_BARO), BARO_MS5611_I2C)
    ap.CFLAGS += -DUSE_I2C2
    ap.srcs += boards/lisa_m/baro_ms5611_i2c.c
  endif
  ap.CFLAGS += -D$(LIA_BARO)

# navgo baro
else ifeq ($(BOARD), navgo)
include $(CFG_SHARED)/spi_master.makefile
ap.CFLAGS += -DUSE_SPI_SLAVE0
ap.CFLAGS += -DUSE_SPI1
ap.srcs += peripherals/mcp355x.c
ap.srcs += $(SRC_BOARD)/baro_board.c

# apogee baro
else ifeq ($(BOARD), apogee)
ap.CFLAGS += -DUSE_I2C1
ap.CFLAGS += -DMPL3115_I2C_DEV=i2c1
ap.CFLAGS += -DMPL3115_ALT_MODE=0
ap.srcs += peripherals/mpl3115.c
ap.srcs += $(SRC_BOARD)/baro_board.c
endif

ifneq ($(BARO_LED),none)
ap.CFLAGS += -DROTORCRAFT_BARO_LED=$(BARO_LED)
endif

#
# Analog Backend
#

ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DUSE_ADC
ap.srcs   += $(SRC_ARCH)/mcu_periph/adc_arch.c
ap.srcs   += subsystems/electrical.c
# baro has variable offset amplifier on booz board
ifeq ($(BOARD), booz)
ap.CFLAGS += -DUSE_DAC
ap.srcs   += $(SRC_ARCH)/mcu_periph/dac_arch.c
endif
else ifeq ($(ARCH), stm32)
ap.CFLAGS += -DUSE_ADC
ap.srcs   += $(SRC_ARCH)/mcu_periph/adc_arch.c
ap.srcs   += subsystems/electrical.c
else ifeq ($(BOARD)$(BOARD_TYPE), ardronesdk)
ap.srcs   += $(SRC_BOARD)/electrical_dummy.c
else ifeq ($(BOARD)$(BOARD_TYPE), ardroneraw)
ap.srcs   += $(SRC_ARCH)/subsystems/electrical/electrical_arch.c
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

ap.srcs += state.c

ap.srcs += $(SRC_FIRMWARE)/stabilization.c
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_none.c
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_rate.c

ap.CFLAGS += -DUSE_NAVIGATION
ap.srcs += $(SRC_FIRMWARE)/guidance/guidance_h.c
ap.srcs += $(SRC_FIRMWARE)/guidance/guidance_h_ref.c
ap.srcs += $(SRC_FIRMWARE)/guidance/guidance_v.c
ap.srcs += $(SRC_FIRMWARE)/guidance/guidance_v_ref.c

#
# INS choice
#
# include subsystems/rotorcraft/ins.makefile
# or
# include subsystems/rotorcraft/ins_extended.makefile
#
# extra:
# include subsystems/rotorcraft/ins_hff.makefile
#

ap.srcs += $(SRC_FIRMWARE)/navigation.c
ap.srcs += subsystems/navigation/common_flight_plan.c

#
# FMS  choice
#
# include booz2_fms_test_signal.makefile
# or
# include booz2_fms_datalink.makefile
# or
# nothing
#
ifeq ($(ARCH), omap)
SRC_FMS=fms
ap.CFLAGS += -I. -I$(SRC_FMS)
ap.srcs   += $(SRC_FMS)/fms_serial_port.c
endif
