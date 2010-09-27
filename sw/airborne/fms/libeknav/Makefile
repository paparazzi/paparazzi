
raw_log_to_ascii: raw_log_to_ascii.c
	gcc -I../../ -std=gnu99 -Wall raw_log_to_ascii.c -o raw_log_to_ascii


fetch_log:
		scp @auto3:/tmp/log_test3.bin .

clean:
	-rm -f *.o *~ *.d
