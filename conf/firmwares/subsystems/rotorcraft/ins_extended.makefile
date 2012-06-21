#
# extended INS with vertical filter using sonar in a better way (flap ground)
#

ap.srcs += $(SRC_SUBSYSTEMS)/ins_extended.c

#  vertical filter float version
ap.srcs += $(SRC_SUBSYSTEMS)/ins/vf_extended_float.c
ap.CFLAGS += -DUSE_VFF_EXTENDED -DDT_VFILTER='(1./$(PERIODIC_FREQUENCY).)'

