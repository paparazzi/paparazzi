
raw_log_to_ascii: raw_log_to_ascii.c
	gcc -I../../ -Wall raw_log_to_ascii.c -o raw_log_to_ascii


fetch_log:
		scp martin@auto3:/tmp/log_test3.bin /home/martin/paparazzi3/trunk/sw/airborne/fms/libeknav
	
clean:
	-rm -f *.o *~ *.d
