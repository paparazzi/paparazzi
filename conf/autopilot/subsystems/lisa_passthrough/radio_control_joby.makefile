

# Radio control
pt.CFLAGS += -DUSE_RADIO_CONTROL
pt.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"radio_control/booz_radio_control_joby.h\"
pt.CFLAGS += -DRADIO_CONTROL_JOBY_MODEL_H=\"radio_control/booz_radio_control_joby_9ch.h\"
pt.srcs += $(SRC_BOOZ)/booz_radio_control.c \
           $(SRC_BOOZ)/radio_control/booz_radio_control_joby.c
pt.CFLAGS += -DRADIO_CONTROL_LED=6
pt.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B115200
pt.CFLAGS += -DRADIO_CONTROL_LINK=Uart3


