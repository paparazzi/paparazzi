# Hey Emacs, this is a -*- makefile -*-
#
# Aspirin IMU v2.2
#
# nearly identical with v2.1, only has the MS5611 baro on SPI.
# The Baro CS line is
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


include $(CFG_SHARED)/imu_aspirin_v2_common.makefile

#
# Baro is connected via SPI, so additionally specify the slave select line for it,
# so that it will be unselected at init (baro CS line high)
#
ifeq ($(ARCH), lpc21)
#IMU_ASPIRIN_2_CFLAGS += -DUSE_SPI_SLAVE1
else ifeq ($(ARCH), stm32)
# SLAVE3 is on PC13, which is the baro CS
IMU_ASPIRIN_2_CFLAGS += -DUSE_SPI_SLAVE3
endif

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_ASPIRIN_2_CFLAGS)
$(TARGET).srcs += $(IMU_ASPIRIN_2_SRCS)
endif
