# Hey Emacs, this is a -*- makefile -*-
#
# MPU6000 via SPI and HMC5883 via I2C on the PX4FMU v1.7 board
#
#
# required xml:
#  <section name="IMU" prefix="IMU_">
#
#    <!-- these gyro and accel calib values are the defaults for px4fmu_v1.7 -->
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
  IMU_PX4FMU_CFLAGS  = -DUSE_IMU
endif

include $(CFG_SHARED)/spi_master.makefile

IMU_PX4FMU_CFLAGS += -DIMU_TYPE_H=\"imu/imu_px4fmu.h\"
IMU_PX4FMU_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_PX4FMU_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_px4fmu.c
IMU_PX4FMU_SRCS   += peripherals/mpu60x0.c
IMU_PX4FMU_SRCS   += peripherals/mpu60x0_spi.c
IMU_PX4FMU_CFLAGS += -DUSE_SPI1
IMU_PX4FMU_CFLAGS += -DUSE_SPI_SLAVE0 -DUSE_SPI_SLAVE1 -DUSE_SPI_SLAVE2

# Magnetometer
IMU_PX4FMU_SRCS   += peripherals/hmc58xx.c
IMU_PX4FMU_CFLAGS += -DUSE_I2C2


# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_PX4FMU_CFLAGS)
$(TARGET).srcs += $(IMU_PX4FMU_SRCS)
endif

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
