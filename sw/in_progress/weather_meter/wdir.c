/* This calculates the adc values for the weather meter wind direction */

#include <stdio.h>

int main(void) {

  int i, j, x = 3.84;
  float r[8+1] = { 33000, 8200, 1000, 2200, 3900, 16000, 120000, 64900,  33000 };
  float r1 = 10000, r2;
  int dir[16], dig[16], temp, temp_i, dir_res[18], dig_res[18];

  for (i=0; i<8; i++) {
    r2 = r[i];
    dir[i*2] = i * 450;
    dig[i*2] = (int)((1024*(r2/(r2+r1))) + 0.5);
    printf("DIR = %d, R = %.2f, V = %.2f, DIG = %d\n",
	dir[i*2],
	r2,
	5*(r2/(r2+r1)),
	dig[i*2]);

    r2 = (r[i]*r[i+1])/(r[i]+r[i+1]);
    dir[i*2+1] = i * 450 + 225;
    dig[i*2+1] = (int)(float)(1024*(r2/(r2+r1)) + 0.5);
    printf("DIR = %d, R = %.2f, V = %.2f, DIG = %d\n",
	dir[i*2+1],
	r2,
	5*(r2/(r2+r1)),
	dig[i*2+1]);
  }

  for (j=0; j<16; j++) {
    temp = 0;
    for (i=0; i<16; i++) {
      if (dig[i] > temp) {
        temp = dig[i];
        temp_i = i;
      }
    }
    dig_res[j+1] = dig[temp_i];
    dir_res[j+1] = dir[temp_i];
    dig[temp_i] = 0;
  }

  dir_res[0] = -1,
  dig_res[0] = 1024;
  dir_res[17] = -1,
  dig_res[17] = 0;

  for (i=0; i<18; i++) {
    printf("DIR = %d, DIG = %d\n",
	dir_res[i],
	dig_res[i]);
  }

  for (i=0; i<17; i++) {
    printf("  if (adc_wdir > %d) return %d;\n",
      (dig_res[i] + dig_res[i+1]) / 2,
      dir_res[i]);
  }
  printf("  return -1;\n");
}
