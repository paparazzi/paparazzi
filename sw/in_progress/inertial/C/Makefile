
# Quiet compilation
Q=@

CFLAGS = -g -Wall  $(shell pkg-config glib-2.0 --cflags)
LDFLAGS = $(shell pkg-config glib-2.0 --libs) -lm

%.o: %.c
	gcc -c -Wall $(CFLAGS) -o $@ $<

CFLAGS += $(shell pkg-config gtk+-2.0 --cflags)
LDFLAGS += $(shell pkg-config gtk+-2.0 --libs) -lgtkdatabox

#
#
# TILT
#
#

OBJS_TILT_UKF =	tilt_data.o     \
		tilt_display.o  \
		tilt_utils.o    \
		tilt_ukf.o     	\
		random.c       	\
		ukf.c		\
		linalg.c	\

tilt_ukf: $(OBJS_TILT_UKF)
	gcc  -o $@ $^ $(LDFLAGS)

OBJS_TILT_EKF =	tilt_data.o 	\
		tilt_display.o	\
		tilt_utils.o	\
		tilt_ekf.o     	\
		random.o       	\
		ekf.o		\
		matrix.o        \

#CFLAGS += -DEKF_UPDATE_DISCRETE
CFLAGS += -DEKF_UPDATE_CONTINUOUS
tilt_ekf: $(OBJS_TILT_EKF)
	gcc  -o $@ $^ $(LDFLAGS)


OBJS_TILT_FAST_EKF =	tilt_data.o 	 \
			tilt_display.o	 \
			tilt_utils.o     \
		        random.o       	 \
			tilt_fast_ekf.o

tilt_fast_ekf: $(OBJS_TILT_FAST_EKF)
	gcc  -o $@ $^ $(LDFLAGS)


#
#
# AHRS EULER
#
#

OBJS_AHRS_EULER_EKF = ahrs_euler_ekf.o \
		      ahrs_utils.o     \
		      ahrs_data.o      \
		      ahrs_display.o   \
		      ekf.o            \
		      matrix.o

ahrs_euler_ekf: $(OBJS_AHRS_EULER_EKF)
	gcc  -o $@ $^ $(LDFLAGS)



#
#
# AHRS QUAT
#
#

OBJS_AHRS_QUAT_UKF = ahrs_quat_ukf.o \
		     ahrs_utils.o    \
		     ahrs_data.o     \
		     ahrs_display.o  \
		     ukf.o           \
		     linalg.o

ahrs_quat_ukf: $(OBJS_AHRS_QUAT_UKF)
	gcc  -o $@ $^ $(LDFLAGS)

OBJS_AHRS_QUAT_EKF = ahrs_quat_ekf.o \
		     ahrs_utils.o    \
		     ahrs_data.o     \
		     ahrs_display.o  \
		     ekf.o           \
		     matrix.o

ahrs_quat_ekf: $(OBJS_AHRS_QUAT_EKF)
	gcc  -o $@ $^ $(LDFLAGS)


OBJS_AHRS_QUAT_FAST_EKF = ahrs_quat_fast_ekf_main.o	\
		          ahrs_quat_fast_ekf.o    	\
		     	   ahrs_data.o		        \
		           ahrs_display.o		\
		           ahrs_utils.o		        \


ahrs_quat_fast_ekf: $(OBJS_AHRS_QUAT_FAST_EKF)
	gcc  -o $@ $^ $(LDFLAGS)


#
#
# Test
#
#

OBJS_TEST_MATRIX = 	test_matrix.o 	\
			matrix.o

test_matrix: $(OBJS_TEST_MATRIX)
	gcc  -o $@ $^ $(LDFLAGS)

OBJS_TEST_UKF =  main.o    \
		linalg.o  \
		random.o  \
		ukf.o     \

test_ukf: $(OBJS_TEST_UKF)
	gcc  -o $@ $^ $(LDFLAGS)


clean:
	$(Q)rm -f *~ *.o tilt_ukf tilt_ekf tilt_fast_ekf ahrs_euler_ekf ahrs_quat_ukf ahrs_quat_ekf ahrs_quat_fast_ekf test_matrix test_ukf
