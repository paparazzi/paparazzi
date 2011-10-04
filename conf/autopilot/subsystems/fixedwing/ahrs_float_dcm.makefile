# Hey Emacs, this is a -*- makefile -*-

# attitude estimation for fixedwings via dcm algorithm


ifeq ($(TARGET), ap)

ap.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_dcm.h\"
ap.CFLAGS += -DUSE_AHRS_ALIGNER
ap.CFLAGS += -DUSE_AHRS

ap.srcs   += $(SRC_SUBSYSTEMS)/ahrs.c
ap.srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
ap.srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_float_dcm.c

ifdef AHRS_ALIGNER_LED
  ap.CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

ifdef CPU_LED
  ap.CFLAGS += -DAHRS_CPU_LED=$(CPU_LED)
endif

ifdef AHRS_PROPAGATE_FREQUENCY
else
  AHRS_PROPAGATE_FREQUENCY = 60
endif

ifdef AHRS_CORRECT_FREQUENCY
else
  AHRS_CORRECT_FREQUENCY = 60
endif

ap.CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=$(AHRS_PROPAGATE_FREQUENCY)
ap.CFLAGS += -DAHRS_CORRECT_FREQUENCY=$(AHRS_CORRECT_FREQUENCY)

endif



# since there is currently no SITL sim for the IMU, we use the infrared sim
# and the ahrs_infrared subsystem

ifeq ($(TARGET), sim)

sim.CFLAGS += -DIR_ROLL_NEUTRAL_DEFAULT=0
sim.CFLAGS += -DIR_PITCH_NEUTRAL_DEFAULT=0

sim.CFLAGS += -DUSE_INFRARED
sim.srcs += subsystems/sensors/infrared.c
sim.srcs += subsystems/sensors/infrared_adc.c

sim.srcs += $(SRC_ARCH)/sim_ir.c
sim.srcs += $(SRC_ARCH)/sim_imu.c


sim.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_infrared.h\"
sim.CFLAGS += -DUSE_AHRS

sim.srcs   += $(SRC_SUBSYSTEMS)/ahrs.c
sim.srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_infrared.c

endif

jsbsim.srcs += $(SRC_ARCH)/jsbsim_ir.c
