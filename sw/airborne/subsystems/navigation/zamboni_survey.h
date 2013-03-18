#ifndef ZAMBONI_H
#define ZAMBONI_H

#include "std.h"

//typedef struct {float x; float y;} point2d;
typedef enum {Z_ERR, Z_ENTRY, Z_SEG, Z_TURN1, Z_RET, Z_TURN2} z_survey_stage;


extern bool_t init_zamboni_survey(uint8_t center_wp, uint8_t dir_wp, float sweep_length, float sweep_spacing, int sweep_lines, float altitude);
extern bool_t zamboni_survey(void);

#endif
