#
# Autopilot
#
ap.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
ap.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/ppm.h\"
ap.CFLAGS += -DRADIO_CONTROL_TYPE_PPM
ap.srcs += $(SRC_SUBSYSTEMS)/radio_control.c                    \
           $(SRC_SUBSYSTEMS)/radio_control/ppm.c  \
           $(SRC_ARCH)/subsystems/radio_control/ppm_arch.c
ap.CFLAGS += -DUSE_TIM2_IRQ

#
# Simulator
#
sim.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_LED=1
sim.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/ppm.h\"
sim.CFLAGS += -DRADIO_CONTROL_TYPE_PPM
sim.srcs += $(SRC_SUBSYSTEMS)/radio_control.c \
            $(SRC_BOOZ)/subsystems/radio_control/ppm.c \
            $(SRC_BOOZ_SIM)/subsystems/radio_control/ppm_arch.c

#
# test_rc_ppm
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#   RADIO_CONTROL_LED
#

