#
# Aspirin IMU
#
#
# required xml:
#  <section name="IMU" prefix="IMU_">
#
#    <define name="GYRO_X_NEUTRAL" value="33924"/>
#    <define name="GYRO_Y_NEUTRAL" value="33417"/>
#    <define name="GYRO_Z_NEUTRAL" value="32809"/>
#
#    <define name="GYRO_X_SENS" value="1.01" integer="16"/>
#    <define name="GYRO_Y_SENS" value="1.01" integer="16"/>
#    <define name="GYRO_Z_SENS" value="1.01" integer="16"/>
#
#    <define name="ACCEL_X_NEUTRAL" value="32081"/>
#    <define name="ACCEL_Y_NEUTRAL" value="33738"/>
#    <define name="ACCEL_Z_NEUTRAL" value="32441"/>
#
#    <define name="ACCEL_X_SENS" value="2.50411474" integer="16"/>
#    <define name="ACCEL_Y_SENS" value="2.48126183" integer="16"/>
#    <define name="ACCEL_Z_SENS" value="2.51396167" integer="16"/>
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

# imu aspirin

IMU_ASPIRIN_CFLAGS  = -DUSE_IMU
IMU_ASPIRIN_CFLAGS += -DIMU_TYPE_H=\"imu/imu_aspirin.h\" -DIMU_OVERRIDE_CHANNELS
IMU_ASPIRIN_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                      $(SRC_SUBSYSTEMS)/imu/imu_aspirin.c \
                      $(SRC_ARCH)/subsystems/imu/imu_aspirin_arch.c

# Magnetometer
IMU_ASPIRIN_SRCS   += peripherals/hmc5843.c $(SRC_ARCH)/peripherals/hmc5843_arch.c

IMU_ASPIRIN_CFLAGS += -DUSE_I2C2

ifeq ($(ARCH), lpc21)
#TODO
else ifeq ($(ARCH), stm32)
IMU_ASPIRIN_CFLAGS += -DUSE_EXTI15_10_IRQ  # Gyro Int on PC14
IMU_ASPIRIN_CFLAGS += -DUSE_EXTI9_5_IRQ    # Mag Int on PB5
IMU_ASPIRIN_CFLAGS += -DUSE_EXTI2_IRQ      # Accel Int on PD2
IMU_ASPIRIN_CFLAGS += -DUSE_DMA1_C4_IRQ    # SPI2 Rx DMA
endif


# Keep CFLAGS/Srcs for imu in separate expression so we can assign it to other targets
# see: conf/autopilot/subsystems/lisa_passthrough/imu_b2_v1.1.makefile for example
ap.CFLAGS += $(IMU_ASPIRIN_CFLAGS)
ap.srcs   += $(IMU_ASPIRIN_SRCS)

# sim not done yet
#sim.CFLAGS += $(IMU_ASPIRIN_CFLAGS)
#sim.srcs   += $(IMU_ASPIRIN_SRCS)
