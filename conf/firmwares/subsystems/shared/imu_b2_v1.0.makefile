# Hey Emacs, this is a -*- makefile -*-
#
# Booz2 IMU booz2v1.0
#
#
# required xml:
#  <section name="IMU" prefix="IMU_">
#
#    <define name="GYRO_X_NEUTRAL" value="33924"/>
#    <define name="GYRO_Y_NEUTRAL" value="33417"/>
#    <define name="GYRO_Z_NEUTRAL" value="32809"/>
#
#    <define name="GYRO_X_SENS" value="1.01" integer="16"/>
#    <define name="GYRO_Y_SENS" value="1.01" integer="16"/>
#    <define name="GYRO_Z_SENS" value="1.01" integer="16"/>
#
#    <define name="ACCEL_X_NEUTRAL" value="32081"/>
#    <define name="ACCEL_Y_NEUTRAL" value="33738"/>
#    <define name="ACCEL_Z_NEUTRAL" value="32441"/>
#
#    <define name="ACCEL_X_SENS" value="2.50411474" integer="16"/>
#    <define name="ACCEL_Y_SENS" value="2.48126183" integer="16"/>
#    <define name="ACCEL_Z_SENS" value="2.51396167" integer="16"/>
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


# common Booz2 IMU files
include $(CFG_SHARED)/imu_b2_common.makefile

# imu Booz2 v1.0
imu_CFLAGS += -DIMU_B2_VERSION_1_0

# Magnetometer
ifndef NO_MAG
imu_CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_AMI601
imu_CFLAGS += -DUSE_AMI601
imu_srcs += peripherals/ami601.c

ifeq ($(ARCH), lpc21)
imu_CFLAGS += -DUSE_I2C1  -DI2C1_SCLL=150 -DI2C1_SCLH=150
else ifeq ($(ARCH), stm32)
#FIXME: untested
imu_CFLAGS += -DUSE_I2C2
endif

endif #NO_MAG

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(imu_CFLAGS)
$(TARGET).srcs += $(imu_srcs)
endif
