
CFLAGS = -Wall                          \
         -I ..                          \
         -I ../../../var/BOOZ           \
         -I ../../airborne              \
         -I ../../include               \
         -I /usr/include/meschach       \
         `pkg-config glib-2.0 --cflags` \

LDFLAGS = -lm                           \
          -lmeschach                    \
          -lpcre                        \
          -lglibivy                     \
          `pkg-config glib-2.0 --libs`  \

SIMDIR = ..

#
#
#

TEST_FDM_SRCS = test_fdm.c                           \
                $(SIMDIR)/booz_flight_model.c        \
                $(SIMDIR)/booz_flight_model_utils.c   

test_fdm : $(TEST_FDM_SRCS)
	gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)


#
#
#

TEST_SENSORS_SRCS = test_sensors.c                            \
                    $(SIMDIR)/booz_flight_model.c             \
                    $(SIMDIR)/booz_flight_model_utils.c       \
                    $(SIMDIR)/booz_sensors_model.c            \
                    $(SIMDIR)/booz_sensors_model_utils.c      \
                    $(SIMDIR)/booz_sensors_model_accel.c      \
                    $(SIMDIR)/booz_sensors_model_gyro.c       \
                    $(SIMDIR)/booz_sensors_model_mag.c        \
                    $(SIMDIR)/booz_sensors_model_rangemeter.c \
                    $(SIMDIR)/booz_sensors_model_baro.c       \
                    $(SIMDIR)/booz_sensors_model_gps.c        \

test_sensors : $(TEST_SENSORS_SRCS)
	gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)
