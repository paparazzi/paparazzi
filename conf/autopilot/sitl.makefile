sim.ARCHDIR = $(ARCH)
sim.CFLAGS += -DSITL -DAP -DFBW -DRADIO_CONTROL -DINTER_MCU -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport -DUSE_INFRARED -DNAV -DUSE_LED -DWIND_INFO
sim.srcs += latlong.c\
            radio_control.c\
            downlink.c\
            commands.c\
            gps.c\
            inter_mcu.c\
            subsystems/sensors/infrared.c\
            $(SRC_FIRMWARE)/stabilization/stabilization_attitude.c\
            $(SRC_FIRMWARE)/guidance/guidance_v.c                 \
            subsystems/nav.c\
            estimator.c\
            sys_time.c\
            $(SRC_FIRMWARE)/main_fbw.c \
            $(SRC_FIRMWARE)/main_ap.c \
            $(SRC_FIRMWARE)/datalink.c \
            $(SRC_ARCH)/ppm_hw.c \
            $(SRC_ARCH)/sim_gps.c\
            $(SRC_ARCH)/sim_ir.c \
            $(SRC_ARCH)/sim_ap.c \
            $(SRC_ARCH)/ivy_transport.c \
            $(SRC_ARCH)/sim_adc_generic.c \
            $(SRC_ARCH)/led_hw.c



