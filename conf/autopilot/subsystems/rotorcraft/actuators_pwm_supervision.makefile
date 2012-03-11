
# add actuatos arch to include directories
ap.CFLAGS += -I$(SRC_FIRMWARE)/actuators/arch/$(ARCH) -DUSE_SERVOS_7AND8

ap.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
ap.srcs += $(SRC_FIRMWARE)/actuators/actuators_pwm_supervision.c
ap.srcs += $(SRC_FIRMWARE)/actuators/arch/$(ARCH)/actuators_pwm_arch.c
