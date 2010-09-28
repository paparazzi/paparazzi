#
# Booz2 IMU booz2v1.0
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

# imu Booz2 v1

# add imu arch to include directories
ap.CFLAGS += -I$(SRC_FIRMWARE)/imu/arch/$(ARCH)

ap.CFLAGS += -DIMU_TYPE_H=\"imu/imu_b2.h\"
ap.CFLAGS += -DIMU_B2_VERSION_1_0
ap.CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_AMI601
ap.CFLAGS += -DSSP_VIC_SLOT=9
ap.srcs += $(SRC_FIRMWARE)/imu.c                   \
           $(SRC_FIRMWARE)/imu/imu_b2.c            \
           $(SRC_FIRMWARE)/imu/arch/$(ARCH)/imu_b2_arch.c

ap.CFLAGS += -DMAX1168_EOC_VIC_SLOT=8
ap.srcs += $(SRC_BOOZ)/peripherals/booz_max1168.c \
           $(SRC_BOOZ_ARCH)/peripherals/booz_max1168_arch.c

ap.CFLAGS += -DUSE_AMI601
ap.srcs += $(SRC_BOOZ)/peripherals/booz_ami601.c
ap.CFLAGS += -DUSE_I2C1  -DI2C1_SCLL=150 -DI2C1_SCLH=150 -DI2C1_VIC_SLOT=11


#
# Simulator
#

# add imu arch to include directories
sim.CFLAGS += -I$(SRC_FIRMWARE)/imu/arch/$(ARCH)

sim.CFLAGS += -DIMU_TYPE_H=\"imu/imu_b2.h\"
sim.CFLAGS += -DIMU_B2_VERSION_1_0
sim.CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_AMI601
sim.srcs += $(SRC_FIRMWARE)/imu.c                   \
           $(SRC_FIRMWARE)/imu/imu_b2.c            \
           $(SRC_FIRMWARE)/imu/arch/$(ARCH)/imu_b2_arch.c

sim.srcs += $(SRC_BOOZ)/peripherals/booz_max1168.c \
            $(SRC_BOOZ_SIM)/peripherals/booz_max1168_arch.c

sim.CFLAGS += -DUSE_AMI601
sim.srcs += $(SRC_BOOZ)/peripherals/booz_ami601.c
sim.CFLAGS += -DUSE_I2C1
