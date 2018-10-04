#ifndef  P_FORWARD
#define P_FORWARD 1.1
#endif

#ifndef  D_FORWARD
#define D_FORWARD 0.0
#endif

#ifndef  P_LATERAL
#define P_LATERAL 1.1
#endif

#ifndef  D_LATERAL
#define D_LATERAL 0.0
#endif

struct dronerace_control_struct
{
  // States
  float psi_ref;    ///< maintain a rate limited speed set-point to smooth heading changes

  // Outputs to inner loop
  float phi_cmd;
  float theta_cmd;
  float psi_cmd;
  float z_cmd;
};

extern struct dronerace_control_struct dr_control;

extern void control_reset(void);
extern void control_run(float dt);

