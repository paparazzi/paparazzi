# Hey Emacs, this is a -*- makefile -*-

# XSens Mti-G

#    <load name="ins_xsens_MTiG_fixedwing.xml">
#      <configure name="XSENS_UART_NR" value="0"/>
#    </load>



#########################################
## ATTITUDE

ifeq ($(TARGET), ap)

#  <init fun="ins_init()"/>
#  <periodic fun="ins_periodic_task()" freq="60"/>
#  <event fun="InsEventCheckAndHandle(handle_ins_msg())"/>
#  <makefile target="ap">
#    <define name="AHRS_TYPE_H" value="\\\"modules/ins/ins_xsens.h\\\"" />
#    <define name="INS_MODULE_H" value="\\\"modules/ins/ins_xsens.h\\\"" />
#    <define name="USE_UART$(XSENS_UART_NR)"/>
#    <define name="INS_LINK" value="Uart$(XSENS_UART_NR)"/>
#    <define name="UART$(XSENS_UART_NR)_BAUD" value="B230400"/>
#    <define name="USE_GPS_XSENS"/>
#    <define name="USE_GPS_XSENS_RAW_DATA" />
#    <define name="GPS_NB_CHANNELS" value="50" />
#    <define name="XSENS_OUTPUT_MODE" value="0x1836" />
#    <file name="ins_xsens.c"/>
#    <define name="AHRS_TRIGGERED_ATTITUDE_LOOP" />
#  </makefile>

# ImuEvent -> XSensEvent
ap.CFLAGS += -DUSE_AHRS -DUSE_INS
ap.CFLAGS += -DIMU_TYPE_H=\"modules/ins/ins_xsens.h\"

# AHRS Results
ap.CFLAGS += -DINS_MODULE_H=\"modules/ins/ins_xsens.h\"
ap.CFLAGS += -DAHRS_TYPE_H=\"modules/ins/ins_xsens.h\"
ap.CFLAGS += -DGPS_TYPE_H=\"modules/ins/ins_xsens.h\"

#B230400
#B115200

ap.CFLAGS += -DUSE_UART$(XSENS_UART_NR)
ap.CFLAGS += -DINS_LINK=Uart$(XSENS_UART_NR)
ifdef XSENS_BAUDRATE_CONFIG
ap.CFLAGS += -DUART$(XSENS_UART_NR)_BAUD=B115200
else
ap.CFLAGS += -DUART$(XSENS_UART_NR)_BAUD=B230400
endif

ap.CFLAGS += -DXSENS_OUTPUT_MODE=0x1836
ap.srcs   += $(SRC_SUBSYSTEMS)/ins.c
ap.srcs   += $(SRC_MODULES)/ins/ins_xsens700.c
ap.CFLAGS += -DAHRS_TRIGGERED_ATTITUDE_LOOP


endif

ifeq ($(TARGET), fbw)

# when compiling FBW only, the settings need to know the AHRS_TYPE

fbw.CFLAGS += -DAHRS_TYPE_H=\"modules/ins/ins_xsens.h\"

endif


#########################################
## GPS

# ap.CFLAGS += -DGPS
ap.CFLAGS += -DUSE_GPS_XSENS
ap.CFLAGS += -DGPS_NB_CHANNELS=50
ap.srcs += $(SRC_SUBSYSTEMS)/gps.c


#########################################
## Simulator
SIM_TARGETS = sim jsbsim nps

ifneq (,$(findstring $(TARGET),$(SIM_TARGETS)))

$(TARGET).CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_sim.h\"
$(TARGET).CFLAGS += -DUSE_AHRS -DAHRS_UPDATE_FW_ESTIMATOR

$(TARGET).srcs   += $(SRC_SUBSYSTEMS)/ahrs.c
$(TARGET).srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_sim.c

$(TARGET).srcs   += $(SRC_SUBSYSTEMS)/ins.c
$(TARGET).srcs   += $(SRC_SUBSYSTEMS)/ins/ins_gps_passthrough.c

$(TARGET).CFLAGS += -DUSE_GPS -DGPS_USE_LATLONG
$(TARGET).CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim.h\"
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/gps.c

endif

