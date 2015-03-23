#include"random.h"
#include<stdlib.h>
#include<math.h>

double
ukf_filter_rand_normal() {
	static int is_ready = 0;
	double norm2,x,y;
	double bm;
	double saved_value;

	if(is_ready == 0) {
		do {
			x = 2.0 * drand48() - 1.0;
			y = 2.0 * drand48() - 1.0;
			norm2 = x * x + y * y;
		} while(norm2 >= 1.0 || norm2 == 0.0);
		bm = sqrt(-2.0 * log(norm2) / norm2);
		saved_value = x * bm;
		is_ready = 1;
		return y * bm;
	} else {
		is_ready = 0;
		return saved_value;
	}
}

