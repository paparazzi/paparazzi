ARCHI=sim

ap.ARCHDIR = $(ARCHI)
ap.ARCH = sitl
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.CFLAGS += -DAP -DFBW
# ap.CFLAGS += -DGPS -DUBX -DINFRARED -DRADIO_CONTROL -DDOWNLINK
#ap.CFLAGS += -DACTUATORS=\"servos_4017.h\" -DSERVOS_4017
#ap.srcs += $(SRC_ARCH)/servos_4017.c
ap.srcs = radio_control.c downlink.c commands.c gps_ubx.c gps.c inter_mcu.c link_mcu.c infrared.c pid.c nav.c estimator.c mainloop.c cam.c sys_time.c main_fbw.c main_ap.c main.c
