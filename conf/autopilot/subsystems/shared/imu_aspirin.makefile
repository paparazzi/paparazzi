
IMU_ASPIRIN_CFLAGS  = -DUSE_IMU
IMU_ASPIRIN_CFLAGS += -DIMU_TYPE_H=\"imu/imu_aspirin.h\" -DIMU_OVERRIDE_CHANNELS
IMU_ASPIRIN_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                      $(SRC_SUBSYSTEMS)/imu/imu_aspirin.c \
                      $(SRC_ARCH)/subsystems/imu/imu_aspirin_arch.c

IMU_ASPIRIN_SRCS   += peripherals/hmc5843.c $(SRC_ARCH)/peripherals/hmc5843_arch.c
IMU_ASPIRIN_CFLAGS += -DUSE_I2C2
IMU_ASPIRIN_SRCS   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
IMU_ASPIRIN_CFLAGS += -DUSE_EXTI15_10_IRQ  # Gyro Int on PC14
IMU_ASPIRIN_CFLAGS += -DUSE_EXTI9_5_IRQ    # Mag Int on PB5
IMU_ASPIRIN_CFLAGS += -DUSE_EXTI2_IRQ      # Accel Int on PD2
IMU_ASPIRIN_CFLAGS += -DUSE_DMA1_C4_IRQ    # SPI2 Rx DMA

ap.CFLAGS += $(IMU_ASPIRIN_CFLAGS)
ap.srcs   += $(IMU_ASPIRIN_SRCS)
