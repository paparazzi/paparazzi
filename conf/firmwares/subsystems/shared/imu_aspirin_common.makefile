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

IMU_ASPIRIN_CFLAGS  = -DIMU_TYPE_H=\"imu/imu_aspirin.h\"
IMU_ASPIRIN_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                      $(SRC_SUBSYSTEMS)/imu/imu_aspirin.c \
                      $(SRC_ARCH)/subsystems/imu/imu_aspirin_arch.c \
                      $(SRC_ARCH)/mcu_periph/spi_arch.c \
                      mcu_periph/spi.c

IMU_ASPIRIN_CFLAGS += -DUSE_SPI -DSPI_MASTER

# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_ASPIRIN_CFLAGS  += -DUSE_IMU
endif

# Gyro
IMU_ASPIRIN_SRCS   += peripherals/itg3200.c

# Magnetometer
#IMU_ASPIRIN_SRCS   += peripherals/hmc5843.c $(SRC_ARCH)/peripherals/hmc5843_arch.c
IMU_ASPIRIN_SRCS   += peripherals/hmc58xx.c

ifeq ($(ARCH), lpc21)
$(error Aspirin driver on lpc is unfinished.)
IMU_ASPIRIN_CFLAGS += -DUSE_SPI_SLAVE0
IMU_ASPIRIN_CFLAGS += -DUSE_SPI1
IMU_ASPIRIN_CFLAGS += -DSSP_VIC_SLOT=9
IMU_ASPIRIN_CFLAGS += -DIMU_ASPIRIN_I2C_DEVICE=i2c1
IMU_ASPIRIN_CFLAGS += -DUSE_I2C1
IMU_ASPIRIN_CFLAGS += -DHMC58XX_I2C_DEVICE=i2c1 -DI2C1_VIC_SLOT=12
else ifeq ($(ARCH), stm32)
IMU_ASPIRIN_CFLAGS += -DIMU_ASPIRIN_I2C_DEVICE=i2c2
IMU_ASPIRIN_CFLAGS += -DUSE_I2C2
IMU_ASPIRIN_CFLAGS += -DUSE_SPI2
# Slave select configuration
# SLAVE2 is on PB12 (NSS) (ADXL345 CS)
IMU_ASPIRIN_CFLAGS += -DUSE_SPI_SLAVE2
endif

include $(CFG_SHARED)/imu_nps.makefile
