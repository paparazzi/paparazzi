#
# simple horizontal filter for INS
#

$(TARGET).CFLAGS += -DUSE_HFF
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/hf_float.c
