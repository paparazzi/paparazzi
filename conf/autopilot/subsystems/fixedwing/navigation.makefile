# Standard fixed wing navigation


#add these to all targets

$(TARGET).CFLAGS += -DNAV
$(TARGET).srcs += $(SRC_FIXEDWING)/nav.c
$(TARGET).srcs += $(SRC_FIXEDWING)/traffic_info.c
$(TARGET).srcs += $(SRC_FIXEDWING)/nav_survey_rectangle.c $(SRC_FIXEDWING)/nav_line.c


