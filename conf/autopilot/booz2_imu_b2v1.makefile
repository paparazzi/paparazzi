# imu Booz2 v1
ap.CFLAGS += -DBOOZ2_IMU_TYPE=\"booz2_imu_b2.h\"
ap.CFLAGS += -DSSP_VIC_SLOT=9
ap.srcs += $(BOOZ_PRIV)/booz2_imu_b2.c $(BOOZ_PRIV_ARCH)/booz2_imu_b2_hw.c
ap.CFLAGS += -DMAX1168_EOC_VIC_SLOT=8
ap.srcs += $(BOOZ_PRIV)/booz2_max1168.c $(BOOZ_PRIV_ARCH)/booz2_max1168_hw.c
ap.CFLAGS += -DUSE_I2C1  -DI2C1_SCLL=150 -DI2C1_SCLH=150 -DI2C1_VIC_SLOT=11 -DI2C1_BUF_LEN=16
#ap.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
ap.CFLAGS += -DUSE_AMI601
ap.srcs += AMI601.c
ap.CFLAGS += -DFLOAT_T=float
ap.srcs += $(BOOZ_PRIV)/booz2_imu.c
