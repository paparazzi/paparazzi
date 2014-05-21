# Hey Emacs, this is a -*- makefile -*-

# Standard fixed wing navigation


#add these to all targets

$(TARGET).CFLAGS += -DNAV
$(TARGET).srcs += $(SRC_FIRMWARE)/nav.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/navigation/common_flight_plan.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/navigation/traffic_info.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/navigation/nav_survey_rectangle.c


