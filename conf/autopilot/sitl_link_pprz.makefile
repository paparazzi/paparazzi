sim.ARCHDIR = $(ARCHI)
sim.ARCH = sitl
sim.TARGET = autopilot
sim.TARGETDIR = autopilot
sim.CFLAGS += -DSITL -DAP -DFBW -DRADIO_CONTROL -DINTER_MCU -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DINFRARED -DRADIO_CONTROL_SETTINGS -DSIM_UART -DDOWNLINK_AP_DEVICE=SimUart -DDOWNLINK_FBW_DEVICE=SimUart
sim.srcs = radio_control.c downlink.c pprz_transport.c commands.c gps.c inter_mcu.c infrared.c pid.c nav.c estimator.c cam.c sys_time.c main_fbw.c main_ap.c rc_settings.c $(SRC_ARCH)/ppm_hw.c $(SRC_ARCH)/sim_gps.c $(SRC_ARCH)/sim_ir.c $(SRC_ARCH)/sim_ap.c  $(SRC_ARCH)/sim_uart.c 
