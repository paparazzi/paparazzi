# joystick for fixedwings

ap.srcs += joystick.c
ap.CFLAGS += -DUSE_JOYSTICK

sim.srcs += joystick.c
sim.CFLAGS += -DUSE_JOYSTICK
