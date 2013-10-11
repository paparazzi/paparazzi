#
# simple with float vertical and horizontal filters for INS
#

$(TARGET).CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_int.h\"
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/ins_int.c

# vertical filter float version
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/vf_float.c

# horizontal filter float version
$(TARGET).CFLAGS += -DUSE_HFF
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/hf_float.c
