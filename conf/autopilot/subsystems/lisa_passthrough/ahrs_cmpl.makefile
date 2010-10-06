#
# Complementary filter for attitude estimation
#

ap.CFLAGS += -DUSE_AHRS_CMPL -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED) -DAHRS_FIXED_POINT
stm_passthrough.srcs += $(SRC_FIRMWARE)/ahrs.c
stm_passthrough.srcs += $(SRC_FIRMWARE)/ahrs/ahrs_aligner.c
stm_passthrough.srcs += $(SRC_FIRMWARE)/ahrs/ahrs_cmpl_euler.c
