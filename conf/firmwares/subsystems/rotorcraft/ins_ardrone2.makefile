#
# simple INS with float vertical filter
#

$(TARGET).CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_ardrone2.h\" -DAUTOPILOT_DISABLE_AHRS_KILL
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/ins_ardrone2.c
