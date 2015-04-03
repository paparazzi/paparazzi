# Hey Emacs, this is a -*- makefile -*-

#
# Analog IMU connected to MCU ADC ports
#
# Only add the configure and define lines for the sensors you actually use.
# E.g. to replace the old gyro_pitch subsystem only add GYRO_P and GYRO_Q
#
#
# <subsystem name="imu" type="analog">
#   <configure name="GYRO_P" value="ADC_0"/>
#   <configure name="GYRO_Q" value="ADC_1"/>
#   <configure name="GYRO_R" value="ADC_2"/>
#   <configure name="ACCEL_X" value="ADC_5"/>
#   <configure name="ACCEL_Y" value="ADC_6"/>
#   <configure name="ACCEL_Z" value="ADC_7"/>
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

ADC_GYRO_NB_SAMPLES ?= 16
ADC_ACCEL_NB_SAMPLES ?= $(ADC_GYRO_NB_SAMPLES)

ifeq ($(ARCH), lpc21)

imu_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_analog.h\"  -DUSE_IMU

imu_CFLAGS += -DADC_CHANNEL_GYRO_NB_SAMPLES=$(ADC_GYRO_NB_SAMPLES)
imu_CFLAGS += -DADC_CHANNEL_ACCEL_NB_SAMPLES=$(ADC_ACCEL_NB_SAMPLES)

ifneq ($(GYRO_P),)
imu_CFLAGS += -DADC_CHANNEL_GYRO_P=$(GYRO_P) -DUSE_$(GYRO_P)
endif

ifneq ($(GYRO_Q),)
imu_CFLAGS += -DADC_CHANNEL_GYRO_Q=$(GYRO_Q) -DUSE_$(GYRO_Q)
endif

ifneq ($(GYRO_R),)
imu_CFLAGS += -DADC_CHANNEL_GYRO_R=$(GYRO_R) -DUSE_$(GYRO_R)
endif

ifneq ($(ACCEL_X),)
imu_CFLAGS += -DADC_CHANNEL_ACCEL_X=$(ACCEL_X) -DUSE_$(ACCEL_X)
endif

ifneq ($(ACCEL_Y),)
imu_CFLAGS += -DADC_CHANNEL_ACCEL_Y=$(ACCEL_Y) -DUSE_$(ACCEL_Y)
endif

ifneq ($(ACCEL_Z),)
imu_CFLAGS += -DADC_CHANNEL_ACCEL_Z=$(ACCEL_Z) -DUSE_$(ACCEL_Z)
endif

imu_srcs += $(SRC_SUBSYSTEMS)/imu.c
imu_srcs += $(SRC_SUBSYSTEMS)/imu/imu_analog.c

else ifeq ($(ARCH), stm32)

$(error Not implemented for the stm32 yet... should be trivial, just do it...)

endif


# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(imu_CFLAGS)
$(TARGET).srcs += $(imu_srcs)
endif

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
