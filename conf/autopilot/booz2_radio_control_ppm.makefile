ap.CFLAGS += -DUSE_RADIO_CONTROL -DRADIO_CONTROL_LED=1
ap.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"booz_radio_control_ppm.h\"
ap.CFLAGS += -DRADIO_CONTROL_TYPE_PPM
ap.srcs += $(SRC_BOOZ)/booz_radio_control.c \
           $(SRC_BOOZ)/booz_radio_control_ppm.c \
           $(SRC_BOOZ_ARCH)/booz_radio_control_ppm_hw.c