# Hey Emacs, this is a -*- makefile -*-
#
# Common part for Aspirin IMU v2.1 and v2.2
#
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

IMU_ASPIRIN_2_CFLAGS += -DASPIRIN_2_SPI_SLAVE_IDX=SPI_SLAVE0
IMU_ASPIRIN_2_CFLAGS += -DASPIRIN_2_SPI_DEV=spi1

IMU_ASPIRIN_2_CFLAGS += -DUSE_SPI1 -DLISA_S
# Slave select configuration
# SLAVE0 is on PA4 (NSS) (MPU600 CS)
IMU_ASPIRIN_2_CFLAGS += -DUSE_SPI_SLAVE0

# SLAVE1 is on PC4, which is the baro CS
IMU_ASPIRIN_2_CFLAGS += -DUSE_SPI_SLAVE1

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_ASPIRIN_2_CFLAGS)
$(TARGET).srcs += $(IMU_ASPIRIN_2_SRCS)
endif

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
