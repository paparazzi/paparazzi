#include <stdio.h>
#include <inttypes.h>

#include "vor_integer_filters.h"
#include "vor_float_filters.h"

int main(int argc, char** argv) {

  int i;

  for(i=0; i<400; i++) {
    float xn_f = 0.9; //!i ? .9 : 0.;
    uint16_t xn_i = xn_f * VIF_SFACT;

    //float   yn_f = vor_float_filter_bp_var(xn_f);
    //int32_t yn_i = vor_int_filter_bp_var(xn_i);

    float   yn_f = vor_float_filter_bp_ref(xn_f);
    int32_t yn_i = vor_int_filter_bp_ref(xn_i);

    printf("%d\t%f\t%f\t%d\t%d\t%f\n", 
	   i, xn_f, yn_f, xn_i, yn_i, (float)yn_i / (float)VIF_SFACT);
  }

  return 0;
}
