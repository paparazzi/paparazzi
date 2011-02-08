
# add actuatos arch to include directories
ap.CFLAGS += -I$(SRC_FIRMWARE)/actuators/arch/$(ARCH)
ifndef SERVOS_REFRESH_FREQ
SERVOS_REFRESH_FREQ=200
endif
ap.CFLAGS += -DSERVO_HZ=$(SERVOS_REFRESH_FREQ)

ap.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
ap.srcs += $(SRC_FIRMWARE)/actuators/actuators_pwm_supervision.c
ap.srcs += $(SRC_FIRMWARE)/actuators/arch/$(ARCH)/actuators_pwm_arch.c
