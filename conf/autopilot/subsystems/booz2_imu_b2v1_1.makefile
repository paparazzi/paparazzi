#
# Booz2 IMU booz2v1
#
#
# required xml:
#  <section name="IMU" prefix="IMU_">
#
#    <define name="GYRO_X_CHAN" value="1"/>
#    <define name="GYRO_Y_CHAN" value="0"/>
#    <define name="GYRO_Z_CHAN" value="2"/>
#
#    <define name="GYRO_X_NEUTRAL" value="33924"/>
#    <define name="GYRO_Y_NEUTRAL" value="33417"/>
#    <define name="GYRO_Z_NEUTRAL" value="32809"/>
#
#    <define name="GYRO_X_SENS" value=" 1.01" integer="16"/>
#    <define name="GYRO_Y_SENS" value="-1.01" integer="16"/>
#    <define name="GYRO_Z_SENS" value="-1.01" integer="16"/>
# 
#    <define name="ACCEL_X_CHAN" value="3"/>
#    <define name="ACCEL_Y_CHAN" value="5"/>
#    <define name="ACCEL_Z_CHAN" value="6"/>    
#
#    <define name="ACCEL_X_NEUTRAL" value="32081"/>
#    <define name="ACCEL_Y_NEUTRAL" value="33738"/>
#    <define name="ACCEL_Z_NEUTRAL" value="32441"/>
#
#    <define name="ACCEL_X_SENS" value="-2.50411474" integer="16"/>
#    <define name="ACCEL_Y_SENS" value="-2.48126183" integer="16"/>
#    <define name="ACCEL_Z_SENS" value="-2.51396167" integer="16"/>
#
#    <define name="MAG_X_CHAN" value="4"/>
#    <define name="MAG_Y_CHAN" value="0"/>
#    <define name="MAG_Z_CHAN" value="2"/>
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

# imu Booz2 v1.1
ap.CFLAGS += -DBOOZ_IMU_TYPE_H=\"imu/booz_imu_b2.h\"
ap.CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_MS2001
ap.CFLAGS += -DSSP_VIC_SLOT=9
ap.srcs += $(SRC_BOOZ)/booz_imu.c                   \
           $(SRC_BOOZ)/imu/booz_imu_b2.c            \
           $(SRC_BOOZ_ARCH)/imu/booz_imu_b2_arch.c

ap.CFLAGS += -DMAX1168_EOC_VIC_SLOT=8
ap.srcs += $(SRC_BOOZ)/peripherals/booz_max1168.c \
           $(SRC_BOOZ_ARCH)/peripherals/booz_max1168_arch.c

ap.CFLAGS += -DMS2001_DRDY_VIC_SLOT=11
ap.srcs += $(SRC_BOOZ)/peripherals/booz_ms2001.c \
           $(SRC_BOOZ_ARCH)/peripherals/booz_ms2001_arch.c


#
# Simulator
#

sim.CFLAGS += -DBOOZ_IMU_TYPE_H=\"imu/booz_imu_b2.h\"
sim.CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_AMI601
sim.srcs += $(SRC_BOOZ)/booz_imu.c                 \
            $(SRC_BOOZ)/imu/booz_imu_b2.c          \
            $(SRC_BOOZ_SIM)/imu/booz_imu_b2_arch.c


sim.srcs += $(SRC_BOOZ)/peripherals/booz_max1168.c \
            $(SRC_BOOZ_SIM)/peripherals/booz_max1168_arch.c

sim.CFLAGS += -DUSE_AMI601
sim.srcs += $(SRC_BOOZ)/peripherals/booz_ami601.c
sim.CFLAGS += -DUSE_I2C1
