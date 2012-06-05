# Hey Emacs, this is a -*- makefile -*-
#
# Aspirin IMU v2.1
#
#
# required xml:
#  <section name="IMU" prefix="IMU_">
#
#    <!-- these gyro and accel calib values are the defaults for aspirin2.1 -->
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

IMU_ASPIRIN_CFLAGS  = -DUSE_IMU
IMU_ASPIRIN_CFLAGS += -DIMU_TYPE_H=\"imu/imu_aspirin2.h\" -DIMU_OVERRIDE_CHANNELS
IMU_ASPIRIN_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                      $(SRC_SUBSYSTEMS)/imu/imu_aspirin2.c \
                      $(SRC_ARCH)/mcu_periph/spi_arch.c \
                      mcu_periph/spi.c

IMU_ASPIRIN_CFLAGS += -DUSE_SPI

ifeq ($(ARCH), lpc21)
#TODO
$(error Not implemented for the LCP21x yet. Needs the new SPI mcu_periph. See issue 147!)
else ifeq ($(ARCH), stm32)
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
