# Hey Emacs, this is a -*- makefile -*-
#
# The IMU system integrated into Lisa/MX V2.1 based on Aspirin V2.2. Major
# difference is that the orientation of the chips is bit different and we need
# to compensate for that.
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

IMU_ASPIRIN_2_CFLAGS += -DASPIRIN_2_SPI_SLAVE_IDX=SPI_SLAVE2
IMU_ASPIRIN_2_CFLAGS += -DASPIRIN_2_SPI_DEV=spi2

IMU_ASPIRIN_2_CFLAGS += -DUSE_SPI2 -DLISA_M_OR_MX_21
# Slave select configuration
# SLAVE2 is on PB12 (NSS) (MPU600 CS)
IMU_ASPIRIN_2_CFLAGS += -DUSE_SPI_SLAVE2

# SLAVE3 is on PC13, which is the baro CS
IMU_ASPIRIN_2_CFLAGS += -DUSE_SPI_SLAVE3

ap.CFLAGS += $(IMU_ASPIRIN_2_CFLAGS)
ap.srcs   += $(IMU_ASPIRIN_2_SRCS)

test_imu.CFLAGS += $(IMU_ASPIRIN_2_CFLAGS)
test_imu.srcs   += $(IMU_ASPIRIN_2_SRCS)

test_ahrs.CFLAGS += $(IMU_ASPIRIN_2_CFLAGS)
test_ahrs.srcs   += $(IMU_ASPIRIN_2_SRCS)

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
