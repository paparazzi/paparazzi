
// Datatype
struct dronerace_ransac_struct
{
    // Settings
    float dt_max;       ///< Maximum time that a vision sample can stay in the fitting buffer
    float dt_novision;    ///< Maximum time that no vision updates are received before the ransac is reset

    // States
    int buf_index_of_last;  ///< index of the last vision sample in the rolling buffer
    int buf_size;       ///< nr of elements in the rolling vision sample buffer

    float corr_x;       ///< result of RANSAC: a correction on the current state
    float corr_y;
    float corr_vx;
    float corr_vy;

    int ransac_cnt;     ///< How many times was RANSAC run
};

// Variables
extern struct dronerace_ransac_struct dr_ransac;


// Reset
extern void ransac_reset(void);

// Correct the state predictions with the help of
void correct_state(void);

// On new IMU measurement: PREDICT
extern void ransac_propagate(void);

// On new vision update: PUSH a measurement update
extern void ransac_push(float time, float x, float y, float mx, float my);
