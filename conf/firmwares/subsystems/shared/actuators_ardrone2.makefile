
# Actuator drivers for the raw ardrone version are included here

$(TARGET).CFLAGS += -DACTUATORS
$(TARGET).srcs   += $(SRC_BOARD)/actuators_ardrone2_raw.c
