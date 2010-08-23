# extra fixed wing navigation


#add these to all targets

$(TARGET).srcs += $(SRC_FIXEDWING)/nav_cube.c
$(TARGET).srcs += $(SRC_FIXEDWING)/discsurvey.c
$(TARGET).srcs += $(SRC_FIXEDWING)/OSAMNav.c
$(TARGET).srcs += $(SRC_FIXEDWING)/snav.c

