# attitude estimation for fixedwings via dcm algorithm


$(TARGET).CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_dcm.h\"

ifeq ($(ARCH), lpc21)

ap.CFLAGS += -DUSE_ANALOG_IMU

ap.srcs += $(SRC_SUBSYSTEMS)/ahrs.c
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
ap.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_float_dcm.c

endif

# since there is currently no SITL sim for the Analog IMU, we use the infrared sim

ifeq ($(TARGET), sim)

sim.CFLAGS += -DIR_ROLL_NEUTRAL_DEFAULT=0
sim.CFLAGS += -DIR_PITCH_NEUTRAL_DEFAULT=0

sim.CFLAGS += -DUSE_INFRARED
sim.srcs += subsystems/sensors/infrared.c

sim.srcs += $(SRC_ARCH)/sim_ir.c
sim.srcs += $(SRC_ARCH)/sim_imu.c

endif

jsbsim.srcs += $(SRC_ARCH)/jsbsim_ir.c
