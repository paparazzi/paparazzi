# Hey Emacs, this is a -*- makefile -*-
#
# Prop1 IMU
#
#
# required xml:
#  <section name="IMU" prefix="IMU_">
#
#    <!-- these gyro and accel calib values are the defaults for prop1 -->
#    <define name="GYRO_X_NEUTRAL" value="0"/>
#    <define name="GYRO_Y_NEUTRAL" value="0"/>
#    <define name="GYRO_Z_NEUTRAL" value="0"/>
#
#    <define name="ACCEL_X_NEUTRAL" value="0"/>
#    <define name="ACCEL_Y_NEUTRAL" value="0"/>
#    <define name="ACCEL_Z_NEUTRAL" value="0"/>
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
  IMU_PROP1_CFLAGS  = -DUSE_IMU
endif

IMU_PROP1_CFLAGS += -DIMU_TYPE_H=\"imu/imu_prop1.h\"
IMU_PROP1_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                    $(SRC_SUBSYSTEMS)/imu/imu_prop1.c

# Accelerometer via I2C
IMU_PROP1_SRCS   += peripherals/adxl345_i2c.c

# Gyro
IMU_PROP1_SRCS   += peripherals/l3g4200.c

# Magnetometer
IMU_PROP1_SRCS   += peripherals/hmc58xx.c

ifeq ($(ARCH), lpc21)
IMU_PROP1_CFLAGS += -DIMU_PROP1_I2C_DEV=i2c1
IMU_PROP1_CFLAGS += -DUSE_I2C1
IMU_PROP1_CFLAGS += -DI2C1_VIC_SLOT=12
else ifeq ($(ARCH), stm32)
IMU_PROP1_CFLAGS += -DUSE_I2C2
endif

IMU_PROP1_CFLAGS += -DIMU_PROP1

# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
# see: conf/autopilot/subsystems/lisa_passthrough/imu_b2_v1.1.makefile for example

ap.CFLAGS += $(IMU_PROP1_CFLAGS)
ap.srcs   += $(IMU_PROP1_SRCS)

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
