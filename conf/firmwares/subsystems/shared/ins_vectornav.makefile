# Vectornav INS Driver
# 2015, Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
# Utah State University, http://aggieair.usu.edu/

### AP & NPS Targets

#
# INS defines
#
VN_CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_vectornav_wrapper.h\"

VN_SRCS += $(SRC_SUBSYSTEMS)/ins.c
VN_SRCS += $(SRC_SUBSYSTEMS)/ins/ins_vectornav.c
VN_SRCS += subsystems/ins/ins_vectornav_wrapper.c

#
# GPS defines
#
VN_CFLAGS += -DUSE_GPS
VN_SRCS += $(SRC_SUBSYSTEMS)/gps.c
ifdef SECONDARY_GPS
ifneq (,$(findstring $(SECONDARY_GPS), vectornav))
# this is the secondary GPS
VN_CFLAGS += -DGPS_SECONDARY_TYPE_H=\"subsystems/ins_vectornav_wrapper.h\"
VN_CFLAGS += -DSECONDARY_GPS=gps_vectornav
else
VN_CFLAGS += -DGPS_TYPE_H=\"subsystems/ins/ins_vectornav_wrapper.h\"
VN_CFLAGS += -DPRIMARY_GPS=gps_vectornav
endif
else
# plain old single GPS usage
VN_CFLAGS += -DGPS_TYPE_H=\"subsystems/ins/ins_vectornav_wrapper.h\"
endif


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
