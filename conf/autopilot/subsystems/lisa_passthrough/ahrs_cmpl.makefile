#
# Complementary filter for attitude estimation
#

ap.CFLAGS += -DUSE_AHRS_CMPL -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
stm_passthrough.srcs += $(SRC_SUBSYSTEMS)/ahrs.c
stm_passthrough.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c
stm_passthrough.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_int_cmpl_euler.c
