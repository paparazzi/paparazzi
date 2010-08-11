# to output standard PPM to a R/C receiver
#
# This is used in the case where you want to directly drive a receiver which has a microcontroller
# to do the decoding and driving of the servos (not a 4015 or 4017 decoder chip).
# The PPM is output on the SERV_CLK pin. The PPM frame rate, pulse width, and number of channels
# can be adjusted in the "servos_ppm_hw.h" file to suit your particular receiver.

ap.CFLAGS += -DACTUATORS=\"servos_ppm_hw.h\" -DSERVOS_PPM_MAT
ap.srcs += $(SRC_FIXEDWING_ARCH)/servos_ppm_hw.c $(SRC_FIXEDWING)/actuators.c

