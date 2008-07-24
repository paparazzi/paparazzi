#include <math.h>

int main(void) {

  /* Put your VFP test code here */
  {
    volatile float i;
    int j;

    for(j = 0; j < 1000000; j++) {
    i = sin(0.1);
    i = cos(0.1);
    i = tan(0.1);
    i = sin(0.2);
    i = cos(0.2);
    i = tan(0.2);
    i = sin(0.3);
    i = cos(0.3);
    i = tan(0.3);
    i = sin(0.4);
    i = cos(0.4);
    i = tan(0.4);
    i = sin(0.5);
    i = cos(0.5);
    i = tan(0.5);
    i = sin(0.6);
    i = cos(0.6);
    i = tan(0.6);
    i = sin(0.7);
    i = cos(0.7);
    i = tan(0.7);
    i = sin(0.8);
    i = cos(0.8);
    i = tan(0.8);
    i = sin(0.9);
    i = cos(0.9);
    i = tan(0.9);
    i = sin(1.0);
    i = cos(1.0);
    i = tan(1.0);
    }
  }
  return 0;
}
