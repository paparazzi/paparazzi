
ap.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
ap.srcs += $(SRC_FIRMWARE)/actuators/actuators_pwm_supervision.c
ap.srcs += $(SRC_ARCH)/subsystems/actuators/actuators_pwm_arch.c

# Simulator
nps.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
nps.srcs += $(SRC_FIRMWARE)/actuators/actuators_pwm_supervision.c
nps.srcs += $(SRC_ARCH)/subsystems/actuators/actuators_pwm_arch.c
