include $(CFG_FIXEDWING)/ahrs_float_cmpl_quat.makefile

$(warning The ahrs_float_cmpl subsystem has been split, please replace <subsystem name="ahrs" type="float_cmpl"/> with <subsystem name="ahrs" type="float_cmpl_quat"/> in your airframe file.)
