
ifndef JSBSIM_INC
JSBSIM_ROOT = /opt/jsbsim
JSBSIM_INC = $(JSBSIM_ROOT)/include/JSBSim
JSBSIM_LIB = $(JSBSIM_ROOT)/lib
endif

jsbsim.ARCHDIR = $(ARCH)

# external libraries
jsbsim.CFLAGS = -I$(SIMDIR) -I/usr/include `pkg-config glib-2.0 --cflags`
jsbsim.LDFLAGS += `pkg-config glib-2.0 --libs` -lm -lpcre -lglibivy -L/usr/lib

# use the paparazzi-jsbsim package if it is installed, otherwise look for JSBsim under /opt/jsbsim
ifndef JSBSIM_PKG
JSBSIM_PKG = $(shell pkg-config JSBSim --exists && echo 'yes')
endif
ifeq ($(JSBSIM_PKG), yes)
	jsbsim.CFLAGS  += `pkg-config JSBSim --cflags`
	jsbsim.LDFLAGS += `pkg-config JSBSim --libs`
else
	JSBSIM_PKG = no
	jsbsim.CFLAGS  += -I$(JSBSIM_INC)
	jsbsim.LDFLAGS += -L$(JSBSIM_LIB) -lJSBSim
endif

jsbsim.CFLAGS += -DSITL -DAP -DFBW -DINTER_MCU -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport -DUSE_INFRARED -DNAV -DUSE_LED -DWIND_INFO -Ifirmwares/fixedwing
jsbsim.srcs = $(SRC_ARCH)/jsbsim_hw.c $(SRC_ARCH)/jsbsim_gps.c $(SRC_ARCH)/jsbsim_ir.c $(SRC_ARCH)/jsbsim_transport.c $(SRC_ARCH)/ivy_transport.c
jsbsim.srcs += latlong.c downlink.c commands.c gps.c inter_mcu.c subsystems/sensors/infrared.c \
               $(SRC_FIXEDWING)/stabilization/stabilization_attitude.c \
               $(SRC_FIXEDWING)/guidance/guidance_v.c\
               subsystems/nav.c estimator.c sys_time.c $(SRC_FIRMWARE)/main_fbw.c $(SRC_FIRMWARE)/main_ap.c $(SRC_FIRMWARE)/datalink.c
jsbsim.srcs += $(SIMDIR)/sim_ac_jsbsim.c
# Choose in your airframe file type of airframe
# jsbsim.srcs += $(SIMDIR)/sim_ac_fw.c
# jsbsim.srcs += $(SIMDIR)/sim_ac_booz.c

#jsbsim.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/ppm.h\" -DRADIO_CONTROL_TYPE_PPM
#jsbsim.srcs	+= radio_control.c radio_control/ppm.c $(SRC_ARCH)/radio_control/ppm_arch.c
