#
# simple horizontal filter for INS
#

ap.CFLAGS += -DUSE_HFF
ap.srcs += $(SRC_FIRMWARE)/ins/hf_float.c

sim.CFLAGS += -DUSE_HFF
sim.srcs += $(SRC_FIRMWARE)/ins/hf_float.c
