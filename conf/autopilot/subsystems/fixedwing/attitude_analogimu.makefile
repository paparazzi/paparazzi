# attitude via analog imu


ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DANALOG_IMU
ap.CFLAGS += -DADC -DUSE_$(GYRO_P) -DUSE_$(GYRO_Q) -DUSE_ADC_$(GYRO_R)
ap.CFLAGS += -DUSE_ADC_3 -DUSE_$(ACCEL_X)  -DUSE_$(ACCEL_y) -DUSE_$(ACCEL_Z)

ap.CFLAGS += -DADC_CHANNEL_GYRO_P=$(GYRO_P) -DADC_CHANNEL_GYRO_Q=$(GYRO_Q) -DADC_CHANNEL_GYRO_R=$(GYRO_R)
ap.CFLAGS += -DADC_CHANNEL_ACCEL_X=$(ACCEL_X) -DADC_CHANNEL_ACCEL_Y=$(ACCEL_Y) -DADC_CHANNEL_ACCEL_Z=$(ACCEL_Z)

ap.srcs += $(SRC_SUBSYSTEMS)/imu/imu_analog.c

ap.srcs += $(SRC_SUBSYSTEMS)/ahrs/dcm/dcm.c
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs/dcm/analogimu.c
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs/dcm/analogimu_util.c

endif

# since there is currently no SITL sim for the Analog IMU, we use the infrared sim 

ifeq ($(TARGET), sim)

sim.CFLAGS += -DIR_ROLL_NEUTRAL_DEFAULT=0
sim.CFLAGS += -DIR_PITCH_NEUTRAL_DEFAULT=0

$(TARGET).CFLAGS += -DUSE_INFRARED
$(TARGET).srcs += subsystems/sensors/infrared.c

sim.srcs += $(SRC_ARCH)/sim_ir.c
sim.srcs += $(SRC_ARCH)/sim_analogimu.c

endif

jsbsim.srcs += $(SRC_ARCH)/jsbsim_ir.c
