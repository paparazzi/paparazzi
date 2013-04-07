

# Radio control
stm_passthrough.CFLAGS += -DRADIO_CONTROL
stm_passthrough.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"radio_control/booz_radio_control_joby.h\"
stm_passthrough.CFLAGS += -DRADIO_CONTROL_JOBY_MODEL_H=\"radio_control/booz_radio_control_joby_9ch.h\"
stm_passthrough.srcs += $(SRC_SUBSYSTEMS)/radio_control.c \
           $(SRC_BOOZ)/radio_control/booz_radio_control_joby.c
stm_passthrough.CFLAGS += -DRADIO_CONTROL_LED=6
stm_passthrough.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B115200
stm_passthrough.CFLAGS += -DRADIO_CONTROL_LINK=UART3


