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


#
# IO defines
#
VN_PORT ?= UART3
VN_BAUD ?= B921600

# Chibios checks
ifneq (,$(findstring USE_CHIBIOS_RTOS,$($(TARGET).CFLAGS)))
# $(info ********* ChibiOS ********)
# Now check which serial port are we using
# UART1
ifneq (,$(findstring UART1,$(VN_PORT)))
#$(info ********* UART1 ********)
VN_PORT = UARTD1
else
# UART2
ifneq (,$(findstring UART2,$(VN_PORT)))
#$(info ********* UART2 ********)
VN_PORT = UARTD2
else
# UART3
ifneq (,$(findstring UART3,$(VN_PORT)))
#$(info ********* UART3 ********)
VN_PORT = UARTD3
else
# UART4
ifneq (,$(findstring UART4,$(VN_PORT)))
#$(info ********* UART4 ********)
VN_PORT = UARTD4
else
# UART5
ifneq (,$(findstring UART5,$(VN_PORT)))
#$(info ********* UART5 ********)
VN_PORT = UARTD5
else
ifneq (,$(findstring UART6,$(VN_PORT)))
#$(info ********* UART6 ********)
VN_PORT = UARTD6
else
# otherwise
$(info ********* Warning: Unknown Vector Nav serial port! ********)
endif # UART6
endif # UART5
endif # UART4
endif # UART3
endif # UART2
endif # UART1
VN_CFLAGS += -DVN_PORT=$(VN_PORT)
VN_CFLAGS += -DVN_BAUD=$(VN_BAUD)
VN_CFLAGS += -DUSE_$(VN_PORT)
else # non-chibios
VN_CFLAGS += -DUSE_$(VN_PORT) -D$(VN_PORT)_BAUD=$(VN_BAUD)
VN_PORT_LOWER=$(shell echo $(VN_PORT) | tr A-Z a-z)
VN_CFLAGS += -DVN_PORT=$(VN_PORT_LOWER)
endif # is chibi os
VN_SRCS   += peripherals/vn200_serial.c

#
# Add to the target (AP+NPS))
# add it for all targets except sim and fbw
#
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(VN_CFLAGS)
$(TARGET).srcs += $(VN_SRCS)
endif
