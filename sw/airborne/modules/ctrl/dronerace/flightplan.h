// for jungle gate:
#define UPPER_GATE 0
#define LOWER_GATE 1
extern int flagHighOrLowGate;

struct dronerace_fp_struct
{
  // Current Gate Position
  int gate_nr;
  float gate_x;
  float gate_y;
  float gate_z;
  float gate_psi;
  float gate_speed;

  // Current Navigation Target
  float x_set;
  float y_set;
  float psi_set;
  float z_set;
};


# define MAX_DETECTION 2

struct jungle_gate_struct
{
    int flagJungleGateDetected;
    int numJungleGateDetection;
    float jungleGateHeight;
    float sumJungleGateHeight;
    float jungleGateDetectionZ[MAX_DETECTION];
    float jungleGateDetectionY[MAX_DETECTION];
    int flagInJungleGate;
    long long timeStartJungleGate;
};

// Variables
extern struct dronerace_fp_struct dr_fp;

#define MAX_GATES 12

#define  REGULAR 0
#define JUNGLE 1
#define VIRTUAL 2
#define DIAL 3

#define BRAKE 1
#define NO_BRAKE 0

struct dronerace_flightplan_item_struct
{
    float x;
    float y;
    float z;
    float psi;
    float speed;
    int type;
    int brake;
    float distance_after_gate;
    int both_side;
};

extern const struct dronerace_flightplan_item_struct gates[MAX_GATES];
extern struct dronerace_flightplan_item_struct waypoints_dr[MAX_GATES];
extern struct jungle_gate_struct jungleGate;

// Functions
extern void flightplan_reset(void);
extern void flightplan_run(void);
extern void resetJungleGate(void);
extern void flightplan_list(void);
