

#ifndef READ_MATRIX_SERIAL_H
#define READ_MATRIX_SERIAL_H

#include "math/pprz_algebra_float.h"

//variables for settings
//OA_general
extern float dx_ref;
extern float dy_ref;
//Vector Method
extern float F1;
extern float F2;
extern float Cfreq;
extern float Ko;
extern float Kg;
extern float Dist_offset;
extern int8_t dis_treshold;
//Potential Method
extern float K_goal;
extern float K_obst;
extern float b_damp;
extern float c1_oa;
extern float c2_oa;
extern float c3_oa;
extern float c4_oa;
extern float c5_oa;
extern float kv;
extern float epsilon;

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
