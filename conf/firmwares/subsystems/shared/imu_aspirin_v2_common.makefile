# Hey Emacs, this is a -*- makefile -*-
#
# Common part for Aspirin IMU v2.1 and v2.2
#
# Use <define name="ASPIRIN_2_DISABLE_MAG value="TRUE"/> to disable the mag.
#
# required xml:
#  <section name="IMU" prefix="IMU_">
#
#    <!-- these gyro and accel calib values are the defaults for aspirin2.1/2.2 -->
#    <define name="GYRO_X_NEUTRAL" value="0"/>
#    <define name="GYRO_Y_NEUTRAL" value="0"/>
#    <define name="GYRO_Z_NEUTRAL" value="0"/>
#
#    <define name="GYRO_X_SENS" value="4.359" integer="16"/>
#    <define name="GYRO_Y_SENS" value="4.359" integer="16"/>
#    <define name="GYRO_Z_SENS" value="4.359" integer="16"/>
#
#    <define name="ACCEL_X_NEUTRAL" value="0"/>
#    <define name="ACCEL_Y_NEUTRAL" value="0"/>
#    <define name="ACCEL_Z_NEUTRAL" value="0"/>
#
#    <define name="ACCEL_X_SENS" value="4.905" integer="16"/>
#    <define name="ACCEL_Y_SENS" value="4.905" integer="16"/>
#    <define name="ACCEL_Z_SENS" value="4.905" integer="16"/>
#
#    <!-- replace the mag calibration with your own-->
#    <define name="MAG_X_NEUTRAL" value="-45"/>
#    <define name="MAG_Y_NEUTRAL" value="334"/>
#    <define name="MAG_Z_NEUTRAL" value="7"/>
#
#    <define name="MAG_X_SENS" value="3.4936416" integer="16"/>
#    <define name="MAG_Y_SENS" value="3.607713" integer="16"/>
#    <define name="MAG_Z_SENS" value="4.90788848" integer="16"/>
#
#  </section>
#
#


# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_ASPIRIN_2_CFLAGS  = -DUSE_IMU
endif

IMU_ASPIRIN_2_CFLAGS += -DIMU_TYPE_H=\"imu/imu_aspirin_2_spi.h\"
IMU_ASPIRIN_2_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_ASPIRIN_2_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_aspirin_2_spi.c
IMU_ASPIRIN_2_SRCS   += peripherals/mpu60x0.c
IMU_ASPIRIN_2_SRCS   += peripherals/mpu60x0_spi.c

include $(CFG_SHARED)/spi_master.makefile

#
# SPI device and slave select defaults
#
ifeq ($(ARCH), lpc21)
ASPIRIN_2_SPI_DEV ?= spi1
ASPIRIN_2_SPI_SLAVE_IDX ?= SPI_SLAVE0
else ifeq ($(ARCH), stm32)
# Slave select configuration
# SLAVE2 is on PB12 (NSS) (MPU600 CS)
ASPIRIN_2_SPI_DEV ?= spi2
ASPIRIN_2_SPI_SLAVE_IDX ?= SPI_SLAVE2
endif

ifeq ($(TARGET), ap)
ifndef ASPIRIN_2_SPI_DEV
$(error Error: ASPIRIN_2_SPI_DEV not configured!)
endif
ifndef ASPIRIN_2_SPI_SLAVE_IDX
$(error Error: ASPIRIN_2_SPI_SLAVE_IDX not configured!)
endif
endif

ASPIRIN_2_SPI_DEV_UPPER=$(shell echo $(ASPIRIN_2_SPI_DEV) | tr a-z A-Z)
ASPIRIN_2_SPI_DEV_LOWER=$(shell echo $(ASPIRIN_2_SPI_DEV) | tr A-Z a-z)

IMU_ASPIRIN_2_CFLAGS += -DUSE_$(ASPIRIN_2_SPI_DEV_UPPER)
IMU_ASPIRIN_2_CFLAGS += -DASPIRIN_2_SPI_DEV=$(ASPIRIN_2_SPI_DEV_LOWER)

IMU_ASPIRIN_2_CFLAGS += -DUSE_$(ASPIRIN_2_SPI_SLAVE_IDX)
IMU_ASPIRIN_2_CFLAGS += -DASPIRIN_2_SPI_SLAVE_IDX=$(ASPIRIN_2_SPI_SLAVE_IDX)

# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
# and re-use that in the imu_aspirin_v2.1 and imu_aspirin_v2.2 makefiles


#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
