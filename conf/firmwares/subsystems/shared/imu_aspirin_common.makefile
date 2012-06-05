# Hey Emacs, this is a -*- makefile -*-
#
# Common part for all Aspirin IMUs
#
# if ACCEL and GYRO SENS/NEUTRAL are not defined,
# the defaults from the datasheet will be used
#
# required xml:
#  <section name="IMU" prefix="IMU_">
#
#    <define name="MAG_X_NEUTRAL" value="2358"/>
#    <define name="MAG_Y_NEUTRAL" value="2362"/>
#    <define name="MAG_Z_NEUTRAL" value="2119"/>
#
#    <define name="MAG_X_SENS" value="3.4936416" integer="16"/>
#    <define name="MAG_Y_SENS" value="3.607713" integer="16"/>
#    <define name="MAG_Z_SENS" value="4.90788848" integer="16"/>
#
#  </section>
#
#

# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_ASPIRIN_CFLAGS  = -DUSE_IMU
endif

IMU_ASPIRIN_CFLAGS += -DIMU_TYPE_H=\"imu/imu_aspirin.h\"
IMU_ASPIRIN_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                      $(SRC_SUBSYSTEMS)/imu/imu_aspirin.c \
                      $(SRC_ARCH)/subsystems/imu/imu_aspirin_arch.c

# Magnetometer
IMU_ASPIRIN_SRCS   += peripherals/hmc5843.c $(SRC_ARCH)/peripherals/hmc5843_arch.c

IMU_ASPIRIN_CFLAGS += -DUSE_I2C2

ifeq ($(ARCH), lpc21)
$(error The aspirin subsystem (using SPI) is currently not implemnented for the lpc21. Please use the aspirin_i2c subsystem.)
else ifeq ($(ARCH), stm32)
IMU_ASPIRIN_CFLAGS += -DUSE_EXTI15_10_IRQ  # Gyro Int on PC14
IMU_ASPIRIN_CFLAGS += -DUSE_EXTI9_5_IRQ    # Mag Int on PB5
IMU_ASPIRIN_CFLAGS += -DUSE_EXTI2_IRQ      # Accel Int on PD2
IMU_ASPIRIN_CFLAGS += -DUSE_DMA1_C4_IRQ    # SPI2 Rx DMA
endif

include $(CFG_SHARED)/imu_nps.makefile
