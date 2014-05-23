# Hey Emacs, this is a -*- makefile -*-
#
# YAI IMU
#
#
# required xml:
#  <section name="IMU" prefix="IMU_">
#
#    <define name="GYRO_X_NEUTRAL" value="33924"/>
#    <define name="GYRO_Y_NEUTRAL" value="33417"/>
#    <define name="GYRO_Z_NEUTRAL" value="32809"/>
#
#    <define name="GYRO_X_SENS" value=" 1.01" integer="16"/>
#    <define name="GYRO_Y_SENS" value="-1.01" integer="16"/>
#    <define name="GYRO_Z_SENS" value="-1.01" integer="16"/>
#
#    <define name="ACCEL_X_NEUTRAL" value="32081"/>
#    <define name="ACCEL_Y_NEUTRAL" value="33738"/>
#    <define name="ACCEL_Z_NEUTRAL" value="32441"/>
#
#    <define name="ACCEL_X_SENS" value="-2.50411474" integer="16"/>
#    <define name="ACCEL_Y_SENS" value="-2.48126183" integer="16"/>
#    <define name="ACCEL_Z_SENS" value="-2.51396167" integer="16"/>
#
#  </section>
#
#

# common Booz2 IMU files
include $(CFG_SHARED)/imu_b2_common.makefile

# imu YAI v1.0
# no default channels and signs defined yet

# No Magnetometer

ap.srcs += $(imu_srcs)
ap.CFLAGS += $(imu_CFLAGS)

test_imu.srcs += $(imu_srcs)
test_imu.CFLAGS += $(imu_CFLAGS)
