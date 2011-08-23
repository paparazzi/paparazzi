include $(CFG_FIXEDWING)/ahrs_infrared.makefile

$(warning Attitude estimation via infrared has been implemented as an AHRS subsystem now. Please replace <subsystem name="attitude" type="infrared"/> with <subsystem name="ahrs" type="infrared"/> in your airframe file.)
