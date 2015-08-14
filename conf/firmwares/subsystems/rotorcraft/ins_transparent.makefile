# INS Transparent
# 2014, Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
# Utah State University, http://aggieair.usu.edu/
$(TARGET).CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_transparent.h\"
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/ins_transparent.c

