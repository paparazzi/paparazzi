

#ifndef _INTER_THREAD_DATA_H
#define _INTER_THREAD_DATA_H


// Inter-thread communication: Unix Socket

// Data from thread to module
struct CVresults {
  int cnt;          // Number of processed frames

  float Velx;       // Velocity as measured by camera
  float Vely;
  int flow_count;

  float cam_h;      // Debug parameters
  int count;
  float OFx, OFy, dx_sum, dy_sum;
  float diff_roll;
  float diff_pitch;
  float FPS;
};

// Data from module to thread
struct PPRZinfo {
  int cnt;        // IMU msg counter
  float phi;      // roll [rad]
  float theta;    // pitch [rad]
  float agl;      // height above ground [m]
};



#endif
