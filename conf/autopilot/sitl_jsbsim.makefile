SIM_TYPE = JSBSIM
sim.ARCHDIR = $(ARCHI)
sim.ARCH = sitl
sim.TARGET = autopilot
sim.TARGETDIR = autopilot
# external libraries
sim.CFLAGS = -I$(SIMDIR) -I/usr/include `pkg-config glib-2.0 --cflags`
sim.LDFLAGS += `pkg-config glib-2.0 --libs` -lm -lpcre -lglibivy -L/usr/lib -lJSBSim

sim.CFLAGS += -DSITL -DAP -DFBW -DRADIO_CONTROL -DINTER_MCU -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport -DINFRARED -DNAV -DLED -DWIND_INFO
sim.srcs = $(SRC_ARCH)/jsbsim_hw.c $(SRC_ARCH)/jsbsim_gps.c $(SRC_ARCH)/jsbsim_ir.c $(SRC_ARCH)/jsbsim_transport.c $(SRC_ARCH)/ivy_transport.c 
sim.srcs += latlong.c radio_control.c downlink.c commands.c gps.c inter_mcu.c infrared.c fw_h_ctl.c fw_v_ctl.c nav.c estimator.c sys_time.c main_fbw.c main_ap.c datalink.c
sim.srcs += $(SIMDIR)/sim_ac_jsbsim.c
#$(SIMDIR)/sim_ac_fw.c

