# Hey Emacs, this is a -*- makefile -*-

# XSens Mti-G

#    <subsystem name="ins" type="xsens700">
#      <configure name="XSENS_UART_NR" value="0"/>
#      <configure name="XSENS_UART_BAUD" value="B115200"/>
#    </subsystem>


#########################################
## ATTITUDE

ap.CFLAGS += -DUSE_INS_MODULE

# AHRS Results
ap.CFLAGS += -DINS_TYPE_H=\"modules/ins/ins_xsens.h\"

ifndef XSENS_UART_BAUD
	XSENS_UART_BAUD = B115200
endif

ap.CFLAGS += -DUSE_UART$(XSENS_UART_NR)
ap.CFLAGS += -DINS_LINK=uart$(XSENS_UART_NR)
ap.CFLAGS += -DUART$(XSENS_UART_NR)_BAUD=$(XSENS_UART_BAUD)
ap.CFLAGS += -DXSENS_OUTPUT_MODE=0x1836
ap.srcs   += $(SRC_SUBSYSTEMS)/ins.c
ap.srcs   += $(SRC_MODULES)/ins/ins_xsens700.c
ap.CFLAGS += -DAHRS_TRIGGERED_ATTITUDE_LOOP


#########################################
## GPS

ap.CFLAGS += -DUSE_GPS_XSENS
ap.CFLAGS += -DGPS_NB_CHANNELS=50
ap.CFLAGS += -DUSE_GPS
ap.CFLAGS += -DGPS_TYPE_H=\"modules/ins/ins_xsens.h\"
ap.srcs += $(SRC_SUBSYSTEMS)/gps.c


#########################################
## Simulator
SIM_TARGETS = sim nps

ifneq (,$(findstring $(TARGET),$(SIM_TARGETS)))

$(TARGET).CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_sim.h\"
$(TARGET).CFLAGS += -DUSE_AHRS

$(TARGET).srcs   += $(SRC_SUBSYSTEMS)/ahrs.c
$(TARGET).srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_sim.c

$(TARGET).srcs   += $(SRC_SUBSYSTEMS)/ins.c
$(TARGET).CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_gps_passthrough_utm.h\"
$(TARGET).srcs   += $(SRC_SUBSYSTEMS)/ins/ins_gps_passthrough_utm.c

$(TARGET).CFLAGS += -DUSE_GPS -DGPS_USE_LATLONG
$(TARGET).CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim.h\"
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/gps.c

endif

