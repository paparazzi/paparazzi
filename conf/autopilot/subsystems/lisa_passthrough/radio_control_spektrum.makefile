
# Radio control

ifndef RADIO_CONTROL_SPEKTRUM_MODEL
RADIO_CONTROL_SPEKTRUM_MODEL=\"booz/radio_control/booz_radio_control_spektrum_dx7se.h\"
endif

stm_passthrough.CFLAGS += -DUSE_RADIO_CONTROL
stm_passthrough.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"booz/radio_control/booz_radio_control_spektrum.h\"
stm_passthrough.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_MODEL_H=$(RADIO_CONTROL_SPEKTRUM_MODEL)
stm_passthrough.srcs += $(SRC_BOOZ)/booz_radio_control.c \
                        $(SRC_BOOZ)/radio_control/booz_radio_control_spektrum.c
stm_passthrough.srcs += $(SRC_BOOZ_ARCH)/radio_control/booz_radio_control_spektrum_arch.c
stm_passthrough.CFLAGS += -DRADIO_CONTROL_LED=5
stm_passthrough.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B115200
stm_passthrough.CFLAGS += -DRADIO_CONTROL_LINK=Uart3

