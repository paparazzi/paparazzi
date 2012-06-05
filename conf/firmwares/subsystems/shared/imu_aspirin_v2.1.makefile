# Hey Emacs, this is a -*- makefile -*-
#
# Aspirin IMU v2.0
#
#
# if ACCEL and GYRO SENS/NEUTRAL are not defined,
# the defaults from the datasheet will be used
#
# required xml:
#  <section name="IMU" prefix="IMU_">
#
#    <define name="MAG_X_NEUTRAL" value="2358"/>
#    <define name="MAG_Y_NEUTRAL" value="2362"/>
#    <define name="MAG_Z_NEUTRAL" value="2119"/>
#
#    <define name="MAG_X_SENS" value="3.4936416" integer="16"/>
#    <define name="MAG_Y_SENS" value="3.607713" integer="16"/>
#    <define name="MAG_Z_SENS" value="4.90788848" integer="16"/>
#
#  </section>
#
#

IMU_ASPIRIN_CFLAGS  = -DUSE_IMU
IMU_ASPIRIN_CFLAGS += -DIMU_TYPE_H=\"imu/imu_aspirin2.h\" -DIMU_OVERRIDE_CHANNELS
IMU_ASPIRIN_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                      $(SRC_SUBSYSTEMS)/imu/imu_aspirin2.c \
                      $(SRC_ARCH)/subsystems/imu/imu_aspirin2_arch.c \
                      $(SRC_ARCH)/mcu_periph/spi_arch.c \
                      mcu_periph/spi.c

IMU_ASPIRIN_CFLAGS += -DUSE_SPI

ifeq ($(ARCH), lpc21)
#TODO
$(error Not implemented for the LCP21x yet. Not hard, just needs to be done. Patches welcome!)
else ifeq ($(ARCH), stm32)
# IMU_ASPIRIN_CFLAGS += -DUSE_EXTI15_10_IRQ  # Gyro Int on PC14
IMU_ASPIRIN_CFLAGS += -DUSE_DMA1_C4_IRQ    # SPI2 Rx DMA
endif

IMU_ASPIRIN_CFLAGS += -DIMU_ASPIRIN_VERSION_2_1

# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
# see: conf/autopilot/subsystems/lisa_passthrough/imu_b2_v1.1.makefile for example

ap.CFLAGS += $(IMU_ASPIRIN_CFLAGS)
ap.srcs   += $(IMU_ASPIRIN_SRCS)

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
