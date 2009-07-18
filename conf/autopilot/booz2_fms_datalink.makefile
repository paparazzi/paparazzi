ap.CFLAGS += -DUSE_FMS -DBOOZ_FMS_TYPE=BOOZ_FMS_TYPE_DATALINK
ap.srcs += $(SRC_BOOZ)/booz_fms.c
ap.srcs += $(SRC_BOOZ)/impl/booz_fms_datalink.c

