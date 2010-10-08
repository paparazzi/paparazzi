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

#
# param: MAX_1168_DRDY_PORT



# imu Booz2 v1.2

# add imu arch to include directories
imu_CFLAGS += -I$(SRC_FIRMWARE)/imu/arch/$(ARCH)

imu_CFLAGS += -DIMU_TYPE_H=\"imu/imu_b2.h\"
imu_CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_HMC5843
imu_CFLAGS += -DIMU_B2_VERSION_1_2
imu_srcs += $(SRC_FIRMWARE)/imu.c                   \
           $(SRC_FIRMWARE)/imu/imu_b2.c            \
           $(SRC_FIRMWARE)/imu/arch/$(ARCH)/imu_b2_arch.c

imu_srcs += $(SRC_BOOZ)/peripherals/booz_max1168.c \
            $(SRC_BOOZ_ARCH)/peripherals/booz_max1168_arch.c

imu_srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
imu_srcs += $(SRC_BOOZ)/peripherals/booz_hmc5843.c \
            $(SRC_BOOZ_ARCH)/peripherals/hmc5843_arch.c

ifeq ($(ARCH), lpc21)
imu_CFLAGS += -DSSP_VIC_SLOT=9
imu_CFLAGS += -DMAX1168_EOC_VIC_SLOT=8
imu_CFLAGS += -DMS2001_DRDY_VIC_SLOT=11
else ifeq ($(ARCH), stm32)
imu_CFLAGS += -DUSE_SPI2 -DUSE_DMA1_C4_IRQ -DUSE_EXTI2_IRQ -DUSE_SPI2_IRQ
imu_CFLAGS += -DMAX_1168_DRDY_PORT=$(MAX_1168_DRDY_PORT)
imu_CFLAGS += -DMAX_1168_DRDY_PORT_SOURCE=$(MAX_1168_DRDY_PORT_SOURCE)
imu_CFLAGS += -DUSE_I2C2 -DUSE_EXTI9_5_IRQ
endif

# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
# see: conf/autopilot/subsystems/lisa_passthrough/imu_b2_v1.1.makefile for example
ap.CFLAGS += $(imu_CFLAGS)
ap.srcs += $(imu_srcs)

#
# Simulator
#

# add imu arch to include directories
sim.CFLAGS += -I$(SRC_FIRMWARE)/imu/arch/$(ARCH)

sim.CFLAGS += -DIMU_TYPE_H=\"imu/imu_b2.h\"
sim.CFLAGS += -DIMU_B2_VERSION_1_1
sim.CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_AMI601
sim.srcs += $(SRC_FIRMWARE)/imu.c                   \
            $(SRC_FIRMWARE)/imu/imu_b2.c            \
            $(SRC_FIRMWARE)/imu/arch/$(ARCH)/imu_b2_arch.c

sim.srcs += $(SRC_BOOZ)/peripherals/booz_max1168.c \
            $(SRC_BOOZ_SIM)/peripherals/booz_max1168_arch.c

sim.CFLAGS += -DUSE_AMI601
sim.srcs   += $(SRC_BOOZ)/peripherals/booz_ami601.c
sim.CFLAGS += -DUSE_I2C1
