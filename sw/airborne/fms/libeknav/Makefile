
raw_log_to_ascii: raw_log_to_ascii.c
	gcc -I../../ -std=gnu99 -Wall raw_log_to_ascii.c -o raw_log_to_ascii


fetch_log:
		scp @auto1:/tmp/log_test3.bin .
		./raw_log_to_ascii > bla.dat
		scp @auto1:/tmp/log_ins_test3.data .

fetch_raw_data:
		scp @auto1:/home/root/log_ins_test3.data .

LIBEKNAV_SRCS = basic_ins_qkf.cpp           \
                ins_qkf_predict.cpp         \
                ins_qkf_observe_vector.cpp  \
                ins_qkf_observe_gps_pvt.cpp \
                ins_qkf_observe_gps_p.cpp



run_filter_on_log: ./libeknav_from_log.cpp $(LIBEKNAV_SRCS) ../../math/pprz_geodetic_double.c ../../math/pprz_geodetic_float.c
	g++ -I/usr/include/eigen2 -I../.. -I../../../include -I../../../../var/FY  -DOVERO_LINK_MSG_UP=AutopilotMessageVIUp -DOVERO_LINK_MSG_DOWN=AutopilotMessageVIDown -DEKNAV_FROM_LOG_DEBUG -o $@ $^

clean:
	-rm -f *.o *~ *.d
