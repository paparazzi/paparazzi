# Hey Emacs, this is a -*- makefile -*-

# change settings via Remote Control, e.g. tune your aircraft

$(TARGET).srcs += rc_settings.c
$(TARGET).CFLAGS += -DRADIO_CONTROL_SETTINGS
