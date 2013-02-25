# asctec controllers v2
#
# required xml configuration:
#
#  servo section with driver="Asctec"
#  command_laws section to map motor_mixing commands to servos
#

include $(CFG_SHARED)/actuators_asctec.makefile

ap.CFLAGS += -DACTUATORS_ASCTEC_V2_PROTOCOL
