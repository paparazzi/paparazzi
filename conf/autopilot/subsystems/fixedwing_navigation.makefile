# Standard fixed wing navigation

ap.CFLAGS += -DNAV
ap.srcs += $(SRC_FIXEDWING)/nav.c $(SRC_FIXEDWING)/fw_h_ctl.c $(SRC_FIXEDWING)/fw_v_ctl.c
ap.srcs += $(SRC_FIXEDWING)/nav_survey_rectangle.c $(SRC_FIXEDWING)/nav_line.c

sim.srcs += $(SRC_FIXEDWING)/nav_survey_rectangle.c $(SRC_FIXEDWING)/nav_line.c
