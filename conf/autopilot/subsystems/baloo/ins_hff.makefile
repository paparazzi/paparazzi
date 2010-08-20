#
# simple horizontal filter for INS
#

ap.CFLAGS += -DUSE_HFF
ap.srcs += $(SRC_BOOZ)/ins/booz2_hf_float.c

sim.CFLAGS += -DUSE_HFF
sim.srcs += $(SRC_BOOZ)/ins/booz2_hf_float.c
