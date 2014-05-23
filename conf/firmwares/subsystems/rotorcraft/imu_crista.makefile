#
# Rotorcraft IMU crista
#
#
# required xml:
#  <section name="IMU" prefix="IMU_">
#
#    <define name="GYRO_X_CHAN" value="1"/>
#    <define name="GYRO_Y_CHAN" value="0"/>
#    <define name="GYRO_Z_CHAN" value="2"/>
#
#    <define name="GYRO_X_SIGN" value="1"/>
#    <define name="GYRO_Y_SIGN" value="1"/>
#    <define name="GYRO_Z_SIGN" value="1"/>
#
#    <define name="GYRO_X_NEUTRAL" value="33924"/>
#    <define name="GYRO_Y_NEUTRAL" value="33417"/>
#    <define name="GYRO_Z_NEUTRAL" value="32809"/>
#
#    <define name="GYRO_X_SENS" value="1.01" integer="16"/>
#    <define name="GYRO_Y_SENS" value="1.01" integer="16"/>
#    <define name="GYRO_Z_SENS" value="1.01" integer="16"/>
#
#    <define name="ACCEL_X_CHAN" value="3"/>
#    <define name="ACCEL_Y_CHAN" value="5"/>
#    <define name="ACCEL_Z_CHAN" value="6"/>
#
#    <define name="ACCEL_X_SIGN" value="1"/>
#    <define name="ACCEL_Y_SIGN" value="1"/>
#    <define name="ACCEL_Z_SIGN" value="1"/>
#
#    <define name="ACCEL_X_NEUTRAL" value="32081"/>
#    <define name="ACCEL_Y_NEUTRAL" value="33738"/>
#    <define name="ACCEL_Z_NEUTRAL" value="32441"/>
#
#    <define name="ACCEL_X_SENS" value="2.50411474" integer="16"/>
#    <define name="ACCEL_Y_SENS" value="2.48126183" integer="16"/>
#    <define name="ACCEL_Z_SENS" value="2.51396167" integer="16"/>
#
#    <define name="MAG_X_CHAN" value="4"/>
#    <define name="MAG_Y_CHAN" value="0"/>
#    <define name="MAG_Z_CHAN" value="2"/>
#
#    <define name="MAG_X_SIGN" value="1"/>
#    <define name="MAG_Y_SIGN" value="1"/>
#    <define name="MAG_Z_SIGN" value="1"/>
#
#    <define name="MAG_X_NEUTRAL" value="2358"/>
#    <define name="MAG_Y_NEUTRAL" value="2362"/>
#    <define name="MAG_Z_NEUTRAL" value="2119"/>
#
#    <define name="MAG_X_SENS" value="3.4936416" integer="16"/>
#    <define name="MAG_Y_SENS" value="3.607713" integer="16"/>
#    <define name="MAG_Z_SENS" value="4.90788848" integer="16"/>
#    <define name="MAG_45_HACK" value="1"/>
#
#  </section>
#
#

# imu Crista

imu_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_crista.h\"
imu_srcs += $(SRC_SUBSYSTEMS)/imu.c
imu_srcs += $(SRC_SUBSYSTEMS)/imu/imu_crista.c
imu_srcs += $(SRC_ARCH)/subsystems/imu/imu_crista_arch.c

imu_CFLAGS += -DUSE_AMI601
imu_srcs   += peripherals/ami601.c
imu_CFLAGS += -DUSE_I2C1

ifeq ($(ARCH), lpc21)
imu_CFLAGS += -DI2C1_SCLL=150 -DI2C1_SCLH=150 -DI2C1_BUF_LEN=16
else ifeq ($(ARCH), stm32)
#FIXME
endif

# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
ap.CFLAGS += $(imu_CFLAGS)
ap.srcs += $(imu_srcs)

#
# Simulator
#
include $(CFG_SHARED)/imu_nps.makefile
