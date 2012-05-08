# Hey Emacs, this is a -*- makefile -*-

include $(CFG_FIXEDWING)/imu_ppzuav.makefile

ap.CFLAGS += -DASPIRIN_IMU
