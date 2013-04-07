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


# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_ASPIRIN_CFLAGS  = -DUSE_IMU
endif

IMU_ASPIRIN_CFLAGS += -DIMU_TYPE_H=\"imu/imu_aspirin2.h\"
IMU_ASPIRIN_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                      $(SRC_SUBSYSTEMS)/imu/imu_aspirin2.c

include $(CFG_SHARED)/spi_master.makefile

ifeq ($(ARCH), lpc21)
IMU_ASPIRIN_CFLAGS += -DUSE_SPI1
IMU_ASPIRIN_CFLAGS += -DUSE_SPI_SLAVE0
else ifeq ($(ARCH), stm32)
IMU_ASPIRIN_CFLAGS += -DUSE_SPI2
# Slave select configuration
# SLAVE2 is on PB12 (NSS) (MPU600 CS)
IMU_ASPIRIN_CFLAGS += -DUSE_SPI_SLAVE2
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
