# Vectornav INS Driver
# 2015, Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
# Utah State University, http://aggieair.usu.edu/

### AP & NPS Targets

#
# IMU defines
#
VN_CFLAGS += -DUSE_IMU
VN_CFLAGS += -DUSE_IMU_FLOAT
VN_CFLAGS += -DIMU_TYPE_H=\"ins/ins_vectornav.h\"

VN_SRCS   += $(SRC_SUBSYSTEMS)/imu.c

#
# GPS defines
#
VN_CFLAGS += -DUSE_GPS 

VN_SRCS += $(SRC_SUBSYSTEMS)/gps.c

#
# AHRS defines
#
ifdef SECONDARY_AHRS
ifneq (,$(findstring $(SECONDARY_AHRS),ahrs_vectornav))
# this is the secondary AHRS
AHRS_VN_CFLAGS += -DAHRS_SECONDARY_TYPE_H=\"subsystems/ins/ins_vectornav_wrapper.h\"
AHRS_VN_CFLAGS += -DSECONDARY_AHRS=ahrs_vectornav
else
# this is the primary AHRS
AHRS_VN_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ins/ins_vectornav_wrapper.h\"
AHRS_VN_CFLAGS += -DPRIMARY_AHRS=ahrs_vectornav
endif
else
# plain old single AHRS usage
AHRS_VN_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ins/ins_vectornav_wrapper.h\"
endif

AHRS_VN_CFLAGS  += -DUSE_AHRS

AHRS_VN_SRCS   += subsystems/ahrs.c
AHRS_VN_SRCS   += subsystems/ins/ins_vectornav_wrapper.c

#
# Add AHRS defines to the target (AP+NPS))
# add it for all targets except sim and fbw
#
ifeq (,$(findstring $(TARGET),sim fbw))
VN_CFLAGS += $(AHRS_VN_CFLAGS)
VN_SRCS += $(AHRS_VN_SRCS)
endif

#
# INS defines
#
VN_CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_vectornav.h\"

VN_SRCS += $(SRC_SUBSYSTEMS)/ins.c
VN_SRCS += $(SRC_SUBSYSTEMS)/ins/ins_vectornav.c

#
# IO defines
#
VN_PORT ?= UART3
VN_BAUD ?= B921600

VN_CFLAGS += -DUSE_$(VN_PORT) -D$(VN_PORT)_BAUD=$(VN_BAUD)
VN_PORT_LOWER=$(shell echo $(VN_PORT) | tr A-Z a-z)
VN_CFLAGS += -DVN_PORT=$(VN_PORT_LOWER)

VN_SRCS   += peripherals/vn200_serial.c

#
# Add to the target (AP+NPS))
# add it for all targets except sim and fbw
#
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(VN_CFLAGS)
$(TARGET).srcs += $(VN_SRCS)
endif

#
# NPS simulator
#
nps.CFLAGS += -DIMU_TYPE_H=\"imu/imu_nps.h\" -DUSE_IMU
nps.srcs   += $(SRC_SUBSYSTEMS)/imu.c $(SRC_SUBSYSTEMS)/imu/imu_nps.c

nps.CFLAGS += -DUSE_GPS
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c

nps.CFLAGS += $(AHRS_VN_CFLAGS)
nps.srcs += $(AHRS_VN_SRCS)
nps.CFLAGS += -DNPS_BYPASS_AHRS -DNPS_BYPASS_INS

nps.CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_gps_passthrough.h\"
nps.srcs   += $(SRC_SUBSYSTEMS)/ins.c
nps.srcs   += $(SRC_SUBSYSTEMS)/ins/ins_gps_passthrough.c
