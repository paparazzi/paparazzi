# Hey Emacs, this is a -*- makefile -*-

include $(CFG_SHARED)/imu_ppzuav.makefile

ap.CFLAGS += -DASPIRIN_IMU
