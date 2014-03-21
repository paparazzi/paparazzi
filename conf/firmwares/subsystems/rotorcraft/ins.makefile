#
# simple INS with float vertical filter
#

$(TARGET).CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_int.h\"
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/ins_int.c

#  vertical filter float version
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/vf_float.c

ifdef INS_PROPAGATE_FREQUENCY
$(TARGET).CFLAGS += -DINS_PROPAGATE_FREQUENCY=$(INS_PROPAGATE_FREQUENCY)
endif
