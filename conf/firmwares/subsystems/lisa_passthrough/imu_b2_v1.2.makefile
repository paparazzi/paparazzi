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
#    <define name="MAG_X_NEUTRAL" value="2358"/>
#    <define name="MAG_Y_NEUTRAL" value="2362"/>
#    <define name="MAG_Z_NEUTRAL" value="2119"/>
#
#    <define name="MAG_X_SENS" value="-3.4936416" integer="16"/>
#    <define name="MAG_Y_SENS" value=" 3.607713" integer="16"/>
#    <define name="MAG_Z_SENS" value="-4.90788848" integer="16"/>
#    <define name="MAG_45_HACK" value="1"/>
#
#  </section>
#
#

# imu Booz2 v1.2

imu_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_b2.h\"
imu_CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_HMC5843
imu_CFLAGS += -DIMU_B2_VERSION_1_2
imu_srcs += $(SRC_SUBSYSTEMS)/imu.c
imu_srcs += $(SRC_SUBSYSTEMS)/imu/imu_b2.c

imu_srcs += peripherals/max1168.c
imu_srcs += $(SRC_ARCH)/peripherals/max1168_arch.c

imu_srcs += peripherals/hmc5843.c
imu_srcs += $(SRC_ARCH)/peripherals/hmc5843_arch.c

ifeq ($(ARCH), lpc21)
imu_CFLAGS += -DMAX1168_EOC_VIC_SLOT=11
#FIXME ms2100 not used on this imu
imu_CFLAGS += -DMS2100_DRDY_VIC_SLOT=12
else ifeq ($(ARCH), stm32)
imu_CFLAGS += -DUSE_SPI2
imu_CFLAGS += -DUSE_I2C2
endif

# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
stm_passthrough.CFLAGS += $(imu_CFLAGS)
stm_passthrough.srcs += $(imu_srcs)

#
# Simulator
#

sim.CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_b2.h\"
#FIXME, should be HMC5843
sim.CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_AMI601
#FIXME, should be verision 1.2
sim.CFLAGS += -DIMU_B2_VERSION_1_1
sim.srcs += $(SRC_SUBSYSTEMS)/imu.c
sim.srcs += $(SRC_SUBSYSTEMS)/imu/imu_b2.c

sim.srcs += peripherals/max1168.c
sim.srcs += $(SRC_ARCH)/peripherals/max1168_arch.c

sim.CFLAGS += -DUSE_AMI601
sim.srcs   += peripherals/ami601.c
sim.CFLAGS += -DUSE_I2C1
