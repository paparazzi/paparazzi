

#ifndef _INTER_THREAD_DATA_H
#define _INTER_THREAD_DATA_H


// Inter-thread communication: Unix Socket

// Data from thread to module
struct CVresults {
  int cnt;
  int status;
  float FPS;
  float Velx;
  float Vely;
  int flow_count;
  float cam_h;
  int count;
  float OFx, OFy, dx_sum, dy_sum;
  float diff_roll, diff_pitch;
};

// Data from module to thread
struct PPRZinfo {
  int cnt;
  float theta;
  float phi;
  float agl;
};



#endif
