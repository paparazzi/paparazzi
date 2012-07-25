#
# simple INS with float vertical filter
#

$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins.c

#  vertical filter float version
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/vf_float.c
$(TARGET).CFLAGS += -DUSE_VFF -DDT_VFILTER='(1./$(PERIODIC_FREQUENCY).)'

