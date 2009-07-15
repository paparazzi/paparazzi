#
# Booz2 IMU crista
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



ap.CFLAGS += -DBOOZ2_IMU_TYPE_H=\"booz2_imu_crista.h\"
ap.srcs += $(SRC_BOOZ)/booz2_imu_crista.c $(SRC_BOOZ_ARCH)/booz2_imu_crista_hw.c
ap.CFLAGS += -DUSE_I2C1  -DI2C1_SCLL=150 -DI2C1_SCLH=150 -DI2C1_VIC_SLOT=11 -DI2C1_BUF_LEN=16
ap.CFLAGS += -DUSE_AMI601
ap.srcs += AMI601.c
ap.srcs += $(SRC_BOOZ)/booz2_imu.c



sim.CFLAGS += -DBOOZ2_IMU_TYPE_H=\"booz2_imu_crista.h\"
sim.srcs += $(SRC_BOOZ)/booz2_imu.c \
	    $(SRC_BOOZ)/booz2_imu_crista.c \

sim.CFLAGS += -DUSE_I2C1
sim.CFLAGS += -DUSE_AMI601
sim.srcs += AMI601.c