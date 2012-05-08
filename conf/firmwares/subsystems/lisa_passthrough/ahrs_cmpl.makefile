#
# Complementary filter for attitude estimation
#

stm_passthrough.CFLAGS += -DUSE_AHRS_CMPL
ifneq ($(AHRS_ALIGNER_LED),none)
  stm_passthrough.CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif
stm_passthrough.srcs += $(SRC_SUBSYSTEMS)/ahrs.c
stm_passthrough.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
stm_passthrough.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_int_cmpl_euler.c
