
# Radio control

ifndef RADIO_CONTROL_SPEKTRUM_MODEL
RADIO_CONTROL_SPEKTRUM_MODEL=\"booz/radio_control/booz_radio_control_spektrum_dx7se.h\"
endif

pt.CFLAGS += -DUSE_RADIO_CONTROL
pt.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"booz/radio_control/booz_radio_control_spektrum.h\"
pt.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_MODEL_H=$(RADIO_CONTROL_SPEKTRUM_MODEL)
pt.srcs += $(SRC_BOOZ)/booz_radio_control.c \
           $(SRC_BOOZ)/radio_control/booz_radio_control_spektrum.c
pt.CFLAGS += -DRADIO_CONTROL_LED=4
pt.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B115200
pt.CFLAGS += -DRADIO_CONTROL_LINK=Uart3


