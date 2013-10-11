#
# simple INS with float vertical filter
#

$(TARGET).CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_int.h\"
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/ins_int.c

#  vertical filter float version
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/vf_float.c

