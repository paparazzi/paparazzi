#
# simple INS with float vertical filter
#

ap.srcs += $(SRC_SUBSYSTEMS)/ins.c

#  vertical filter float version
ap.srcs += $(SRC_SUBSYSTEMS)/ins/vf_float.c
ap.CFLAGS += -DUSE_VFF -DDT_VFILTER='(1./$(PERIODIC_FREQUENCY).)'

