STAB_RATE_CFLAGS  = -DUSE_STABILIZATION_RATE
STAB_RATE_SRCS  = $(SRC_FIRMWARE)/stabilization/stabilization_indi.c
STAB_RATE_SRCS  = $(SRC_FIRMWARE)/stabilization/stabilization_rate_indi.c

ap.CFLAGS += $(STAB_RATE_CFLAGS)
ap.srcs += $(STAB_RATE_SRCS)

nps.CFLAGS += $(STAB_RATE_CFLAGS)
nps.srcs += $(STAB_RATE_SRCS)
