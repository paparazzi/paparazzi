# Hey Emacs, this is a -*- makefile -*-
#
# simulated IMU for NPS (NewPaparazziSim)
#
# If ACCEL and GYRO SENS/NEUTRAL are not defined,
# the defaults of aspirin v1.5 are used.
# This fits the nps_sensors_params_default.h
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

nps.CFLAGS += -DIMU_TYPE_H=\"imu/imu_nps.h\" -DUSE_IMU
nps.srcs   += $(SRC_SUBSYSTEMS)/imu.c $(SRC_SUBSYSTEMS)/imu/imu_nps.c
