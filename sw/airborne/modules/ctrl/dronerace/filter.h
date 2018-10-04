struct dronerace_vision_struct {
  int cnt;
  float dx;
  float dy;
  float dz;
};

// store for logging purposes
extern struct dronerace_vision_struct dr_vision;

struct dronerace_state_struct {
  // Time
  float time;

  // Positon
  float x;
  float y;

  // Speed
  float vx;
  float vy;

  // Heading
  float psi;

  // Logic
  int assigned_gate_index;
};

extern float log_mx, log_my;

extern struct dronerace_state_struct dr_state;


extern void filter_reset(void);

extern void filter_predict(float phi, float theta, float psi, float dt);
extern void filter_correct(void);
extern float mx;
extern float my;
extern float filteredX;
extern float filteredY;
