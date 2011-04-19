# standard and extra fixed wing navigation


#add these to all targets

$(TARGET).CFLAGS += -DNAV
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/nav.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/navigation/traffic_info.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/navigation/nav_survey_rectangle.c $(SRC_SUBSYSTEMS)/navigation/nav_line.c

$(TARGET).srcs += $(SRC_SUBSYSTEMS)/navigation/nav_cube.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/navigation/discsurvey.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/navigation/OSAMNav.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/navigation/snav.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/navigation/spiral.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/navigation/poly_survey_adv.c

