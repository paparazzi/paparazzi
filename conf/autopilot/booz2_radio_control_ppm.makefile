#
# Autopilot
#
ap.CFLAGS += -DUSE_RADIO_CONTROL -DRADIO_CONTROL_LED=1
ap.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"radio_control/booz_radio_control_ppm.h\"
ap.CFLAGS += -DRADIO_CONTROL_TYPE_PPM
ap.srcs += $(SRC_BOOZ)/booz_radio_control.c                    \
           $(SRC_BOOZ)/radio_control/booz_radio_control_ppm.c  \
           $(SRC_BOOZ_ARCH)/radio_control/booz_radio_control_ppm_arch.c

#
# Simulator
#
sim.CFLAGS += -DUSE_RADIO_CONTROL -DRADIO_CONTROL_LED=1
sim.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"radio_control/booz_radio_control_ppm.h\"
sim.CFLAGS += -DRADIO_CONTROL_TYPE_PPM
sim.srcs += $(SRC_BOOZ)/booz_radio_control.c \
            $(SRC_BOOZ)/radio_control/booz_radio_control_ppm.c \
            $(SRC_BOOZ_SIM)/radio_contrl/booz_radio_control_ppm_arch.c
