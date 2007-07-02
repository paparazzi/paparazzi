
CFLAGS = -g -Wall  `pkg-config glib-2.0 --cflags`
LDFLAGS = `pkg-config glib-2.0 --libs` -lm

%.o: %.c
	gcc -c -Wall $(CFLAGS) -o $@ $<

OBJS =  main.o    \
	linalg.o  \
	random.o  \
	ukf.o     \
	imu.o

test_ukf: $(OBJS)
	gcc  -o $@ $^ $(LDFLAGS)


CFLAGS += `pkg-config gtk+-2.0 --cflags`
LDFLAGS += `pkg-config gtk+-2.0 --libs` -lgtkdatabox 

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

CFLAGS += -DEKF_UPDATE_DISCRETE
#CFLAGS += -DEKF_UPDATE_CONTINUOUS
tilt_ekf: $(OBJS_TILT_EKF)
	gcc  -o $@ $^ $(LDFLAGS) 


OBJS_TILT_EKF_OPTIM =	tilt_data.o 	 \
			tilt_display.o	 \
			tilt_utils.o     \
		        random.o       	 \
			tilt_ekf_optim.o \

tilt_ekf_optim: $(OBJS_TILT_EKF_OPTIM)
	gcc  -o $@ $^ $(LDFLAGS)

#
#
# AHRS
#
#

OBJS_AHRS_UKF = ahrs_ukf.o 	\
		ahrs_utils.o    \
		ahrs_data.o     \
		ahrs_display.o	\
		ukf.o           \
		linalg.o

ahrs_ukf: $(OBJS_AHRS_UKF)
	gcc  -o $@ $^ $(LDFLAGS)

OBJS_AHRS_EKF = ahrs_ekf.o 	\
		ahrs_utils.o    \
		ahrs_data.o     \
		ahrs_display.o	\
		ekf.o           \
		matrix.o

#CFLAGS += -DAHRS
ahrs_ekf: $(OBJS_AHRS_EKF)
	gcc  -o $@ $^ $(LDFLAGS)


OBJS_AHRS_FAST_EKF = ahrs_fast_ekf_run.o	\
		     ahrs_fast_ekf.o    	\
		     ahrs_data.o		\
		     ahrs_display.o		\
		     ahrs_utils.o		\


ahrs_fast_ekf: $(OBJS_AHRS_FAST_EKF)
	gcc  -o $@ $^ $(LDFLAGS)


#
#
# Tests
#
#

OBJS_TEST_MATRIX = 	test_matrix.o 	\
			matrix.o

test_matrix: $(OBJS_TEST_MATRIX)
	gcc  -o $@ $^ $(LDFLAGS) 

clean:
	rm -f *~ *.o test_ukf tilt_ukf tilt_ekf tilt_ekf_optim ahrs_ukf ahrs_ekf ahrs_fast_ekf test_matrix