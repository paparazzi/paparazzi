ap.CFLAGS += -DUSE_FMS -DBOOZ_FMS_TYPE=BOOZ_FMS_TYPE_DATALINK
ap.srcs += $(SRC_BOOZ)/booz_fms.c
ap.srcs += $(SRC_BOOZ)/fms/booz_fms_datalink.c

sim.CFLAGS += -DUSE_FMS -DBOOZ_FMS_TYPE=BOOZ_FMS_TYPE_DATALINK
sim.srcs += $(SRC_BOOZ)/booz_fms.c
sim.srcs += $(SRC_BOOZ)/fms/booz_fms_datalink.c

