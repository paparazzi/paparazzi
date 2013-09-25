#
# extended INS with vertical filter using sonar in a better way (flap ground)
#

$(TARGET).CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_int_extended.h\"
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/ins_int_extended.c

#  vertical filter float version
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/vf_extended_float.c
$(TARGET).CFLAGS += -DUSE_VFF_EXTENDED

