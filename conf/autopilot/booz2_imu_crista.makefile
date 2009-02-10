



ap.CFLAGS += -DBOOZ2_IMU_TYPE=\"booz2_imu_crista.h\"
ap.srcs += $(SRC_BOOZ)/booz2_imu_crista.c $(SRC_BOOZ_ARCH)/booz2_imu_crista_hw.c
ap.CFLAGS += -DUSE_I2C1  -DI2C1_SCLL=150 -DI2C1_SCLH=150 -DI2C1_VIC_SLOT=11 -DI2C1_BUF_LEN=16
ap.CFLAGS += -DUSE_AMI601
ap.srcs += AMI601.c
ap.CFLAGS += -DFLOAT_T=float
ap.srcs += $(SRC_BOOZ)/booz2_imu.c