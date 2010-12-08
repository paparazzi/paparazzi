# attitude via analog imu


ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DANALOG_IMU -DADC -DUSE_ADC_0 -DUSE_ADC_1 -DUSE_ADC_2 -DUSE_ADC_3 -DUSE_ADC_4  -DUSE_ADC_5 -DUSE_ADC_6 -DUSE_ADC_7 

ap.srcs += $(SRC_FIXEDWING)/subsystems/analogimu/dcm.c $(SRC_FIXEDWING)/subsystems/analogimu/arduimu.c $(SRC_FIXEDWING)/subsystems/analogimu/matrix.c $(SRC_FIXEDWING)/subsystems/analogimu/vector.c

ap.srcs += $(SRC_FIXEDWING)/subsystems/analogimu/analogimu.c $(SRC_FIXEDWING)/subsystems/analogimu/analogimu_util.c
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
