#
# Analog IMU connected to MCU ADC ports
#
#
# <subsystem name="imu" type="analog">
#   <param name="GYRO_P" value="ADC_0"/>
#   <param name="GYRO_Q" value="ADC_1"/>
#   <param name="GYRO_R" value="ADC_2"/>
#   <param name="ACCEL_X" value="ADC_5"/>
#   <param name="ACCEL_Y" value="ADC_6"/>
#   <param name="ACCEL_Z" value="ADC_7"/>
# </subsystem>
#
# required xml:
# <section name="IMU" prefix="IMU_">
#
#    <define name="GYRO_P_NEUTRAL" value="512"/>
#    <define name="GYRO_Q_NEUTRAL" value="512"/>
#    <define name="GYRO_R_NEUTRAL" value="512"/>
#
#    <define name="GYRO_P_SENS" value="0.017" integer="16"/>
#    <define name="GYRO_Q_SENS" value="0.017" integer="16"/>
#    <define name="GYRO_R_SENS" value="0.017" integer="16"/>
#
#    <define name="GYRO_P_SIGN" value="1" />
#    <define name="GYRO_Q_SIGN" value="1" />
#    <define name="GYRO_R_SIGN" value="-1" />
#
#    <define name="ACCEL_X_SENS" value="0.1" integer="16"/>
#    <define name="ACCEL_Y_SENS" value="0.1" integer="16"/>
#    <define name="ACCEL_Z_SENS" value="0.1" integer="16"/>
#
#    <define name="ACCEL_X_NEUTRAL" value="512"/>
#    <define name="ACCEL_Y_NEUTRAL" value="512"/>
#    <define name="ACCEL_Z_NEUTRAL" value="512"/>
#
#    <define name="ACCEL_X_SIGN" value="1"/>
#    <define name="ACCEL_Y_SIGN" value="-1"/>
#    <define name="ACCEL_Z_SIGN" value="1"/>
#
#  </section>
#


ifeq ($(ARCH), lpc21)

imu_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_analog.h\"  -DUSE_IMU

imu_CFLAGS += -DADC
imu_CFLAGS += -DUSE_$(GYRO_P) -DUSE_$(GYRO_Q) -DUSE_$(GYRO_R)
imu_CFLAGS += -DUSE_$(ACCEL_X) -DUSE_$(ACCEL_Y) -DUSE_$(ACCEL_Z)

imu_CFLAGS += -DADC_CHANNEL_GYRO_P=$(GYRO_P) -DADC_CHANNEL_GYRO_Q=$(GYRO_Q) -DADC_CHANNEL_GYRO_R=$(GYRO_R)
imu_CFLAGS += -DADC_CHANNEL_ACCEL_X=$(ACCEL_X) -DADC_CHANNEL_ACCEL_Y=$(ACCEL_Y) -DADC_CHANNEL_ACCEL_Z=$(ACCEL_Z)

imu_srcs += $(SRC_SUBSYSTEMS)/imu.c
imu_srcs += $(SRC_SUBSYSTEMS)/imu/imu_analog.c

endif

# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
# see: conf/autopilot/subsystems/lisa_passthrough/imu_b2_v1.1.makefile for example
ap.CFLAGS += $(imu_CFLAGS)
ap.srcs += $(imu_srcs)
