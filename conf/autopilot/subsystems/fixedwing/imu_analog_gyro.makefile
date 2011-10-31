# Hey Emacs, this is a -*- makefile -*-

#
# Analog roll (and optionally pitch) gyros connected to MCU ADC ports
#
# To use a roll gyro only:
# <subsystem name="imu" type="analog_gyro">
#   <configure name="GYRO_P" value="ADC_3"/>
# </subsystem>
#
# To use roll and pitch gyros:
# <subsystem name="imu" type="analog_gyro">
#   <configure name="GYRO_P" value="ADC_3"/>
#   <configure name="GYRO_Q" value="ADC_4"/>
# </subsystem>
#
#
# required xml:
# <section name="IMU" prefix="IMU_">
#
#    <define name="GYRO_P_NEUTRAL" value="512"/>
#    <define name="GYRO_Q_NEUTRAL" value="512"/>
#
#    <define name="GYRO_P_SENS" value="0.017" integer="16"/>
#    <define name="GYRO_Q_SENS" value="0.017" integer="16"/>
#
#    <define name="GYRO_P_SIGN" value="1" />
#    <define name="GYRO_Q_SIGN" value="1" />
#
#  </section>
#


ifeq ($(ARCH), lpc21)

imu_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_analog_gyro.h\"  -DUSE_IMU

imu_CFLAGS += -DADC -DADC_CHANNEL_GYRO_NB_SAMPLES=$(ADC_GYRO_NB_SAMPLES)

ifneq ($(GYRO_P),)
imu_CFLAGS += -DADC_CHANNEL_GYRO_P=$(GYRO_P) -DUSE_$(GYRO_P)
endif

ifneq ($(GYRO_Q),)
imu_CFLAGS += -DADC_CHANNEL_GYRO_Q=$(GYRO_Q) -DUSE_$(GYRO_Q)
endif

#ifneq ($(GYRO_P_TEMP),)
#imu_CFLAGS += -DADC_CHANNEL_GYRO_P_TEMP=$(GYRO_P_TEMP) -DUSE_$(GYRO_P_TEMP)
#endif


imu_srcs += $(SRC_SUBSYSTEMS)/imu.c
imu_srcs += $(SRC_SUBSYSTEMS)/imu/imu_analog_gyro.c

endif

# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
# see: conf/autopilot/subsystems/lisa_passthrough/imu_b2_v1.1.makefile for example
ap.CFLAGS += $(imu_CFLAGS)
ap.srcs += $(imu_srcs)
