
ap.CFLAGS += -DUSE_RADIO_CONTROL -DRADIO_CONTROL_LED=1
ap.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"impl/booz_radio_control_spektrum.h\"
ap.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_MODEL_H=\"impl/booz_radio_control_spektrum_dx7se.h\"
ap.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B115200
ap.CFLAGS += -DRADIO_CONTROL_LINK=Uart0
ap.srcs += $(SRC_BOOZ)/booz_radio_control.c \
           $(SRC_BOOZ_IMPL)/booz_radio_control_spektrum.c \
