

#ifndef READ_MATRIX_SERIAL_H
#define READ_MATRIX_SERIAL_H

#include "math/pprz_algebra_float.h"

//variables for settings
//OA_general
extern float dx_ref;//reference x position wrt initial postion
extern float dy_ref;//reference y position wrt initial postion
//Vector Method
extern float F1;//Gain independent of obstalce heading
extern float F2;//Gain depenent on obstalce heading
extern float Cfreq;//rate at which the size of a vector decreases when the angle wrt the flight path angle increases
extern float Ko;//Gain of the obstacle vector
extern float Kg;//Gain of the goal vector
extern float Dist_offset;
extern int8_t dis_treshold;
//Potential Method
extern float K_goal;//Gain of the goal potential
extern float K_obst;//Gain of the obstacle potential
extern float b_damp;//Damping term
extern float c1_oa;//used to balance the attraction of a distant goal with near obstalces
extern float c2_oa;//ensures that a very distant goal still has effect
extern float c3_oa;//controls 'gap shooting'behaviour
extern float c4_oa;//Increasing this parameter decreased the effect of distant obstacles
extern float c5_oa;//increases obstacle potential to infinity when an obstalc is approached
extern float kv;//Rate at which translational velocity is decreased
extern float epsilon;//Distance from goal coordinate at which the goal is considered reached

extern void serial_init(void);
extern void serial_update(void);
extern void serial_start(void);
void setAnglesMeasurements(float *anglesMeasurements, float *anglesMeasurements_ver, float *centersensorRad, float *fieldOfViewRad,
                           uint16_t *size_matrix_local);
extern void matrix_2_pingpong(float *distancesMeters, uint16_t *size_matrix, float *distances_hor);
extern void kalman_filter_distances(float *distancesMeters);
extern void butterworth_filter_distances(float *distancesMeters);
extern void CN_calculate_target(void);
extern void pingpong_euler(float *distances_hor, float *horizontalAnglesMeasurements,
                           int horizontalAmountOfMeasurements);
extern void potential_field_heading(float *distancesMeters, float *anglesMeasurements, uint16_t *size_matrix_local, struct FloatVect2 goal_diff);
extern void vector_field_velocity(float *distancesMeters, float *anglesMeasurements, float *anglesMeasurements_ver,uint16_t *size_matrix_local, struct FloatVect2 goal_diff);
extern void logic_based_velocity(float *distancesMeters, float *angle_hor_board_local, float *stereo_fow_local ,uint16_t *size_matrix_local, struct FloatVect2 goal_diff);
void vector_and_logic_velocity(float *distancesMeters, float *anglesMeasurements, float *anglesMeasurements_ver, float *angle_hor_board_local, float *stereo_fow_local, uint16_t *size_matrix_local, struct FloatVect2 goal_diff);


#endif
