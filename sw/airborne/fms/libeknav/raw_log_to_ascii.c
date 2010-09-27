

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "math/pprz_algebra_float.h"

// fixme,put that in a header
struct raw_log_entry {
  double time;
  struct FloatRates   gyro;
  struct FloatVect3   accel;
  struct FloatVect3   mag;
};

int main(int argc, char** argv) {

  const char* filename = "log_test3.bin";
  int raw_log_fd = open(filename, O_RDONLY); 
  // if (fd==-1) blaaa
  
  while (1) {
    struct raw_log_entry e;
    ssize_t nb_read = read(raw_log_fd, &e, sizeof(e));
    if (nb_read != sizeof(e)) break;
    printf("%f GYRO %f %f %f\n", e.time, e.gyro.p, e.gyro.q, e.gyro.r);
  }

  return 0;
}
