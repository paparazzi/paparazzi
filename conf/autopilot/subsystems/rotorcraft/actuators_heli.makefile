
# add actuatos arch to include directories
ap.CFLAGS += -I$(SRC_FIRMWARE)/actuators/arch/$(ARCH)
ap.CFLAGS += -DUSE_HELI

ap.srcs += $(SRC_FIRMWARE)/actuators/actuators_heli.c
ap.srcs += $(SRC_FIRMWARE)/actuators/arch/$(ARCH)/actuators_pwm_arch.c
