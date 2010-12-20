include $(PAPARAZZI_SRC)/conf/autopilot/twin_avr.makefile

LOCAL_CFLAGS += -DCTL_BRD_V1_2 -DBOARD_CONFIG=\"v1_2.h\"
