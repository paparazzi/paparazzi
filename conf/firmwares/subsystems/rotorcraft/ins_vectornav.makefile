#
# simple with float vertical and horizontal filters for INS
# Universal Serial Driver
#
VN_PORT ?= UART3
VN_BAUD ?= B921600

VN_CFLAGS += -DUSE_$(VN_PORT) -D$(VN_PORT)_BAUD=$(VN_BAUD)
VN_PORT_LOWER=$(shell echo $(VN_PORT) | tr A-Z a-z)
VN_CFLAGS += -DVN_PORT=$(VN_PORT_LOWER)

VN_CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_vectornav.h\"
VN_srcs += $(SRC_SUBSYSTEMS)/ins.c
VN_srcs += $(SRC_SUBSYSTEMS)/ins/ins_vectornav.c

ap.CFLAGS += $(VN_CFLAGS)
ap.srcs += $(VN_srcs)

#
# NPS Target
#

