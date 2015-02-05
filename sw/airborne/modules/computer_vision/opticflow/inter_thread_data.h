

#ifndef _INTER_THREAD_DATA_H
#define _INTER_THREAD_DATA_H


// Inter-thread communication: Unix Socket

// Data from thread to module
struct CVresults {
  int cnt;
  int status;
  float Velx;
  float Vely;
  int flow_count;
};

// Data from module to thread
struct PPRZinfo {
  int cnt;
  float theta;
  float phi;
  float agl;
};



#endif
