
ap.CFLAGS += -DUSE_RADIO_CONTROL
ap.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"radio_control/booz_radio_control_dummy.h\"
ap.srcs += $(SRC_BOOZ)/booz_radio_control.c $(SRC_BOOZ)/radio_control/booz_radio_control_dummy.c
sim.CFLAGS += -DUSE_RADIO_CONTROL
sim.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"radio_control/booz_radio_control_dummy.h\"
sim.srcs += $(SRC_BOOZ)/booz_radio_control.c $(SRC_BOOZ)/radio_control/booz_radio_control_dummy.c
