sim.ARCHDIR = $(ARCHI)
sim.ARCH = sitl
sim.TARGET = autopilot
sim.TARGETDIR = autopilot
sim.CFLAGS += -DSITL -DAP -DFBW -DRADIO_CONTROL -DINTER_MCU -DDOWNLINK -DDOWNLINK_TRANSPORT=XBeeTransport -DINFRARED -DRADIO_CONTROL_SETTINGS -DSIM_UART -DXBEE_UART=SimUart
sim.srcs = radio_control.c downlink.c xbee.c commands.c gps.c inter_mcu.c infrared.c pid.c nav.c estimator.c cam.c sys_time.c main_fbw.c main_ap.c rc_settings.c $(SRC_ARCH)/ppm_hw.c $(SRC_ARCH)/sim_gps.c $(SRC_ARCH)/sim_ir.c $(SRC_ARCH)/sim_ap.c  $(SRC_ARCH)/sim_uart.c 
