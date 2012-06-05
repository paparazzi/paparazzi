
# add actuatos arch to include directories
$(TARGET).CFLAGS += -I$(SRC_FIRMWARE)/actuators/arch/$(ARCH)

ap.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
ap.srcs += $(SRC_FIRMWARE)/actuators/actuators_pwm_supervision.c
ap.srcs += $(SRC_FIRMWARE)/actuators/arch/$(ARCH)/actuators_pwm_arch.c

# Simulator
nps.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
nps.srcs += $(SRC_FIRMWARE)/actuators/actuators_pwm_supervision.c
nps.srcs += $(SRC_FIRMWARE)/actuators/arch/$(ARCH)/actuators_pwm_arch.c
