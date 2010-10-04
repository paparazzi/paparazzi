
raw_log_to_ascii: raw_log_to_ascii.c
	gcc -I../../ -std=gnu99 -Wall raw_log_to_ascii.c -o raw_log_to_ascii


fetch_log:
		scp @auto1:/tmp/log_ins_test3.data .
		scp @auto1:/tmp/log_test3.bin .
		./raw_log_to_ascii > bla.dat

clean:
	-rm -f *.o *~ *.d
