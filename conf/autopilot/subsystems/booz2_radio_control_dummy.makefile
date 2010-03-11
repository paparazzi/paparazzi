RDYB_SRCS += $(SRC_BOOZ)/booz_radio_control.c $(SRC_BOOZ)/radio_control/booz_radio_control_dummy.c
RDYB_CFLAGS += -DUSE_RADIO_CONTROL
RDYB_CFLAGS += -DRADIO_CONTROL_TYPE_H=\"radio_control/booz_radio_control_dummy.h\"
