#
# Booz2 IMU booz2v1.2
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

#
# param: MAX_1168_DRDY_PORT



# imu aspirin

imu_CFLAGS += -DIMU_TYPE_H=\"imu/imu_aspirin.h\" -DIMU_OVERRIDE_CHANNELS
imu_srcs += $(SRC_SUBSYSTEMS)/imu.c
imu_srcs += $(SRC_SUBSYSTEMS)/imu/imu_aspirin.c
imu_srcs += $(SRC_ARCH)/subsystems/imu/imu_aspirin_arch.c
#imu_srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c

#imu_srcs += peripherals/max1168.c
#imu_srcs += $(SRC_ARCH)/peripherals/max1168_arch.c

imu_srcs += peripherals/hmc5843.c
imu_srcs += $(SRC_ARCH)/peripherals/hmc5843_arch.c

imu_CFLAGS += -DUSE_DMA1_C4_IRQ -DUSE_EXTI2_IRQ -DUSE_EXTI15_10_IRQ
imu_CFLAGS += -DUSE_I2C2 -DUSE_EXTI9_5_IRQ

# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
# see: conf/autopilot/subsystems/lisa_passthrough/imu_b2_v1.1.makefile for example
ap.CFLAGS += $(imu_CFLAGS)
ap.srcs += $(imu_srcs)
